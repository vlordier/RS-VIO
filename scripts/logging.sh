#!/usr/bin/env bash
# Shared logging utilities for RS-VIO shell scripts
# Source this file in scripts that need logging functions

# ANSI color codes
readonly COLOR_BLUE='\033[0;34m'
readonly COLOR_GREEN='\033[0;32m'
readonly COLOR_YELLOW='\033[1;33m'
readonly COLOR_RED='\033[0;31m'
# shellcheck disable=SC2034
readonly COLOR_CYAN='\033[0;36m'
readonly NC='\033[0m' # No Color

# Standard logging functions with consistent interface
# All functions accept multiple arguments via $*

log_info() {
    echo -e "${COLOR_BLUE}[INFO]${NC} $*"
}

log_success() {
    echo -e "${COLOR_GREEN}[✓]${NC} $*"
}

log_warn() {
    echo -e "${COLOR_YELLOW}[!]${NC} $*"
}

log_error() {
    echo -e "${COLOR_RED}[✗]${NC} $*"
}

log_header() {
    local width=60
    echo ""
    echo -e "${COLOR_BLUE}$(printf '═%.0s' $(seq 1 $width))${NC}"
    echo -e "${COLOR_BLUE}$*${NC}"
    echo -e "${COLOR_BLUE}$(printf '═%.0s' $(seq 1 $width))${NC}"
}

log_section() {
    echo ""
    echo -e "${COLOR_BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${COLOR_BLUE}║${NC} $*"
    echo -e "${COLOR_BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# Alternative single-argument versions for backward compatibility
log_info_single() {
    echo -e "${COLOR_BLUE}[INFO]${NC} $1"
}

log_success_single() {
    echo -e "${COLOR_GREEN}[✓]${NC} $1"
}

log_warn_single() {
    echo -e "${COLOR_YELLOW}[!]${NC} $1"
}

log_error_single() {
    echo -e "${COLOR_RED}[✗]${NC} $1"
}

log_header_single() {
    local width=60
    echo ""
    echo -e "${COLOR_BLUE}$(printf '═%.0s' $(seq 1 $width))${NC}"
    echo -e "${COLOR_BLUE}$1${NC}"
    echo -e "${COLOR_BLUE}$(printf '═%.0s' $(seq 1 $width))${NC}"
}
