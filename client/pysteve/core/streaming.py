"""
TCP streaming client for high-speed data acquisition from STEVE.
"""

import json
import socket
import threading
import time
from typing import Optional, Callable, Dict, Any, List
from collections import deque

from pysteve.core.exceptions import SteveStreamError, SteveConnectionError


class SteveStreamer:
    """
    TCP streaming client for real-time STEVE data acquisition.
    
    Connects to STEVE's TCP streaming server (port 8888) and receives
    JSON-formatted data packets at configurable rates (10-100 Hz).
    
    Features thread-safe callback invocation with optional locking for
    advanced users who handle their own synchronization.
    
    Args:
        client: SteveClient instance for controlling streaming
        host: STEVE device IP (default: uses client's host)
        port: TCP streaming port (default: 8888)
        threadsafe: Use locks to protect callback invocation (default: True)
        buffer_size: Number of samples to buffer (default: 1000)
        auto_reconnect: Automatically reconnect on connection loss (default: True)
    
    Example:
        >>> from pysteve import SteveClient, SteveStreamer
        >>> 
        >>> client = SteveClient("192.168.1.100")
        >>> streamer = SteveStreamer(client)
        >>> 
        >>> def on_data(sample):
        ...     print(f"Position: {sample['position_deg']:.2f}°")
        >>> 
        >>> streamer.start_stream(interval_ms=20, callback=on_data)
        >>> time.sleep(10)
        >>> streamer.stop_stream()
    """

    def __init__(
        self,
        client: "SteveClient",  # type: ignore
        host: Optional[str] = None,
        port: int = 8888,
        threadsafe: bool = True,
        buffer_size: int = 1000,
        auto_reconnect: bool = True,
    ):
        self.client = client
        self.host = host or client.host
        self.port = port
        self.threadsafe = threadsafe
        self.buffer_size = buffer_size
        self.auto_reconnect = auto_reconnect

        self._socket: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._connected = False
        self._callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self._callback_lock = threading.Lock() if threadsafe else None
        self._buffer: deque = deque(maxlen=buffer_size)
        self._buffer_lock = threading.Lock()
        self._last_seq: Optional[int] = None
        self._dropped_packets = 0

    def register_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """
        Register a callback function to be called for each data sample.
        
        Args:
            callback: Function that accepts a dict containing sample data
        
        Example:
            >>> def my_callback(sample):
            ...     print(f"Torque: {sample['torque_nm']:.3f} Nm")
            >>> streamer.register_callback(my_callback)
        """
        self._callbacks.append(callback)

    def unregister_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """
        Unregister a previously registered callback.
        
        Args:
            callback: Function to remove
        """
        if callback in self._callbacks:
            self._callbacks.remove(callback)

    def start_stream(
        self,
        interval_ms: int = 100,
        callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> None:
        """
        Start TCP streaming.
        
        Args:
            interval_ms: Streaming interval in milliseconds (10-1000)
            callback: Optional callback function for each sample
        
        Raises:
            SteveStreamError: If streaming fails to start
        
        Example:
            >>> streamer.start_stream(interval_ms=20)
        """
        if self._running:
            raise SteveStreamError("Streaming already active")

        if callback:
            self.register_callback(callback)

        # Start streaming on STEVE device
        try:
            self.client.start_stream(interval_ms=interval_ms)
        except Exception as e:
            raise SteveStreamError(f"Failed to start streaming on device: {e}")

        # Connect to TCP port
        try:
            self._connect()
        except Exception as e:
            # Stop streaming on device if connection fails
            try:
                self.client.stop_stream()
            except Exception:
                pass
            raise SteveStreamError(f"Failed to connect to streaming port: {e}")

        # Start receiver thread
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def stop_stream(self) -> None:
        """
        Stop TCP streaming.
        
        Closes connection and stops receiver thread.
        """
        self._running = False

        # Close socket
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
            self._connected = False

        # Wait for thread to terminate
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            self._thread = None

        # Stop streaming on device
        try:
            self.client.stop_stream()
        except Exception:
            pass

        # Clear buffer
        with self._buffer_lock:
            self._buffer.clear()
            self._last_seq = None
            self._dropped_packets = 0

    def _connect(self) -> None:
        """Establish TCP connection to streaming server."""
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(5.0)
        try:
            self._socket.connect((self.host, self.port))
            self._connected = True
            
            # Read hello message
            data = self._socket.recv(1024).decode("utf-8")
            if not data.startswith('{"stream":"valve"'):
                raise SteveStreamError(f"Invalid hello message: {data}")
                
        except socket.timeout:
            raise SteveConnectionError(
                f"Connection timeout to {self.host}:{self.port}"
            )
        except socket.error as e:
            raise SteveConnectionError(
                f"Failed to connect to {self.host}:{self.port}: {e}"
            )

    def _receive_loop(self) -> None:
        """Background thread for receiving and processing data."""
        buffer = ""
        reconnect_attempts = 0
        max_reconnect_attempts = 10 if self.auto_reconnect else 0

        while self._running:
            try:
                if not self._connected:
                    # Attempt reconnection
                    if reconnect_attempts >= max_reconnect_attempts:
                        break
                    
                    try:
                        time.sleep(min(2 ** reconnect_attempts, 30))
                        self._connect()
                        reconnect_attempts = 0
                    except Exception:
                        reconnect_attempts += 1
                        continue

                # Receive data
                data = self._socket.recv(4096).decode("utf-8")
                
                if not data:
                    # Connection closed
                    self._connected = False
                    if self._socket:
                        self._socket.close()
                        self._socket = None
                    continue

                buffer += data

                # Process complete JSON lines
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    
                    if not line:
                        continue

                    try:
                        sample = json.loads(line)
                        self._process_sample(sample)
                    except json.JSONDecodeError:
                        # Invalid JSON - skip
                        pass

            except socket.timeout:
                # No data received - continue
                pass
            except socket.error:
                # Connection error
                self._connected = False
                if self._socket:
                    try:
                        self._socket.close()
                    except Exception:
                        pass
                    self._socket = None
            except Exception:
                # Unexpected error - continue
                pass

    def _process_sample(self, sample: Dict[str, Any]) -> None:
        """
        Process received data sample.
        
        Validates sequence numbers, buffers sample, and invokes callbacks.
        """
        # Check for dropped packets
        seq = sample.get("seq")
        if seq is not None and self._last_seq is not None:
            expected_seq = (self._last_seq + 1) & 0xFFFFFFFF  # 32-bit wrap
            if seq != expected_seq:
                dropped = (seq - expected_seq) & 0xFFFFFFFF
                self._dropped_packets += dropped
        self._last_seq = seq

        # Buffer sample
        with self._buffer_lock:
            self._buffer.append(sample)

        # Invoke callbacks
        if self._callbacks:
            if self._callback_lock:
                # Thread-safe callback invocation
                with self._callback_lock:
                    for callback in self._callbacks:
                        try:
                            callback(sample)
                        except Exception:
                            # Don't let callback errors crash receiver
                            pass
            else:
                # Advanced users handle their own synchronization
                for callback in self._callbacks:
                    try:
                        callback(sample)
                    except Exception:
                        pass

    def get_latest_sample(self) -> Optional[Dict[str, Any]]:
        """
        Get the most recent data sample.
        
        Returns:
            Latest sample dict, or None if no data received
        """
        with self._buffer_lock:
            return self._buffer[-1] if self._buffer else None

    def get_last_n_samples(self, n: int) -> List[Dict[str, Any]]:
        """
        Get the last N data samples.
        
        Args:
            n: Number of samples to retrieve
        
        Returns:
            List of up to N most recent samples
        """
        with self._buffer_lock:
            return list(self._buffer)[-n:]

    def get_all_samples(self) -> List[Dict[str, Any]]:
        """
        Get all buffered samples.
        
        Returns:
            List of all buffered samples
        """
        with self._buffer_lock:
            return list(self._buffer)

    def clear_buffer(self) -> None:
        """Clear the sample buffer."""
        with self._buffer_lock:
            self._buffer.clear()
            self._last_seq = None
            self._dropped_packets = 0

    @property
    def is_streaming(self) -> bool:
        """Check if streaming is active."""
        return self._running and self._connected

    @property
    def dropped_packets(self) -> int:
        """Get number of dropped packets detected."""
        return self._dropped_packets

    @property
    def buffer_usage(self) -> float:
        """
        Get buffer usage as percentage.
        
        Returns:
            Buffer fill percentage (0.0-1.0)
        """
        with self._buffer_lock:
            return len(self._buffer) / self.buffer_size

    def __del__(self) -> None:
        """Cleanup on deletion."""
        if self._running:
            self.stop_stream()
