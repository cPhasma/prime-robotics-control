#!/usr/bin/env bash
set -e
echo "Replace YOUR_LOGIN with your GitHub login before running."
git log --oneline --graph
git remote add origin https://github.com/YOUR_LOGIN/prime-robotics-control.git
git push -u origin main
