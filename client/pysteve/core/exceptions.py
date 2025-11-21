"""
Custom exceptions for PySteve client.
"""


class SteveError(Exception):
    """Base exception for all STEVE-related errors."""

    pass


class SteveConnectionError(SteveError):
    """Raised when connection to STEVE device fails or is lost."""

    pass


class SteveAPIError(SteveError):
    """Raised when STEVE REST API returns an error response."""

    def __init__(self, message: str, status_code: int = None, response_data: dict = None):
        super().__init__(message)
        self.status_code = status_code
        self.response_data = response_data


class SteveValidationError(SteveError):
    """Raised when parameter validation fails."""

    pass


class SteveStreamError(SteveError):
    """Raised when TCP streaming encounters an error."""

    pass


class SteveTimeoutError(SteveError):
    """Raised when an operation times out."""

    pass


class SteveAuthError(SteveError):
    """Raised when authentication fails (invalid API key)."""

    pass
