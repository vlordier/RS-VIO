#!/usr/bin/env python3
"""Shared logging utilities for RS-VIO scripts."""



class Colors:
    """ANSI color codes for terminal output."""
    BLUE = '\033[0;34m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    RED = '\033[0;31m'
    CYAN = '\033[0;36m'
    RESET = '\033[0m'


def log_info(msg: str) -> None:
    """Log an info message with green checkmark."""
    print(f"{Colors.GREEN}✓{Colors.RESET} {msg}")


def log_warn(msg: str) -> None:
    """Log a warning message with yellow warning symbol."""
    print(f"{Colors.YELLOW}⚠{Colors.RESET} {msg}")


def log_error(msg: str) -> None:
    """Log an error message with red X."""
    print(f"{Colors.RED}✗{Colors.RESET} {msg}")


def log_header(msg: str, width: int = 60) -> None:
    """Log a header message with blue border."""
    print()
    print(f"{Colors.BLUE}{'═' * width}{Colors.RESET}")
    print(f"{Colors.BLUE}{msg:^{width}}{Colors.RESET}")
    print(f"{Colors.BLUE}{'═' * width}{Colors.RESET}")


def log_section(msg: str) -> None:
    """Log a section message with cyan arrow."""
    print(f"\n{Colors.CYAN}→ {msg}{Colors.RESET}")


def setup_console_logging(verbose: bool = False) -> None:
    """Setup console logging configuration.

    Args:
        verbose: If True, enable more detailed logging
    """
    # This could be extended to configure logging levels, etc.
    pass
