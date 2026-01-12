#!/bin/bash
cd /Users/vincent/Work/RS-VIO || exit
git add tests/end_to_end_vio_tests.rs
git commit -m 'Strengthen integration tests with realistic data (15/15 passing)'
git status --short
