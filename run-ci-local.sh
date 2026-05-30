#!/bin/bash
set -e

echo "=========================================="
echo "Running local CI checks (mirrors .github/workflows/ci.yml)"
echo "=========================================="

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# [1/3] Formatting
echo -e "\n${YELLOW}[1/3] Checking code formatting (clang-format)...${NC}"
if ! command -v clang-format &> /dev/null; then
    echo -e "${RED}ERROR: clang-format not installed${NC}"
    echo "Install with: sudo apt-get install clang-format"
    exit 1
fi

if find src \( -name '*.cpp' -o -name '*.h' \) ! -name 'UTFT*' | \
    xargs clang-format --dry-run --Werror --style=file; then
    echo -e "${GREEN}✓ Formatting check passed${NC}"
else
    echo -e "${RED}✗ Formatting check failed${NC}"
    echo "Fix with: find src \( -name '*.cpp' -o -name '*.h' \) ! -name 'UTFT*' | xargs clang-format -i --style=file"
    exit 1
fi

# [2/3] Static analysis
echo -e "\n${YELLOW}[2/3] Running static analysis (cppcheck)...${NC}"
if ! command -v cppcheck &> /dev/null; then
    echo -e "${RED}ERROR: cppcheck not installed${NC}"
    echo "Install with: sudo apt-get install cppcheck"
    exit 1
fi

if cppcheck \
    --enable=warning,style,performance \
    --suppress=missingIncludeSystem \
    --suppress=unusedFunction \
    --suppress=syntaxError:src/UTFT.h \
    --inline-suppr \
    --error-exitcode=1 \
    src/main.cpp src/server.h src/display.h; then
    echo -e "${GREEN}✓ Static analysis passed${NC}"
else
    echo -e "${RED}✗ Static analysis failed${NC}"
    exit 1
fi

# [3/3] Build
echo -e "\n${YELLOW}[3/3] Building firmware (ATmega2560)...${NC}"
if ! command -v pio &> /dev/null; then
    echo -e "${RED}ERROR: PlatformIO not installed${NC}"
    echo "Install with: pipx install platformio"
    exit 1
fi

if pio run -e ATmega2560 2>&1 | tee /tmp/pio_output.txt; then
    echo -e "${GREEN}✓ Build passed${NC}"
    echo -e "\n${YELLOW}Memory Usage:${NC}"
    grep -E "^RAM:|^Flash:" /tmp/pio_output.txt || true
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

echo -e "\n${GREEN}=========================================="
echo "All CI checks passed! ✓"
echo "==========================================${NC}"
