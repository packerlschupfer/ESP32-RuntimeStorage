#!/bin/bash
# Switch between main application and I2C scanner

cd "$(dirname "$0")"

if [ "$1" == "scanner" ]; then
    echo "Switching to I2C Scanner mode..."
    if [ -f src/main.cpp ]; then
        mv src/main.cpp src/main.cpp.app
    fi
    if [ -f src/i2c_scanner.cpp ]; then
        mv src/i2c_scanner.cpp src/main.cpp
        echo "Switched to I2C Scanner. Run: pio run -t upload -t monitor"
    else
        echo "Error: i2c_scanner.cpp not found!"
        exit 1
    fi
elif [ "$1" == "app" ]; then
    echo "Switching to Application mode..."
    if [ -f src/main.cpp ] && [ -f src/main.cpp.app ]; then
        mv src/main.cpp src/i2c_scanner.cpp
        mv src/main.cpp.app src/main.cpp
        echo "Switched to Application. Run: pio run -t upload -t monitor"
    else
        echo "Error: Application backup not found!"
        exit 1
    fi
else
    echo "Usage: $0 [scanner|app]"
    echo "  scanner - Switch to I2C scanner mode"
    echo "  app     - Switch back to application mode"
    
    # Show current mode
    if [ -f src/main.cpp ]; then
        if grep -q "I2C Scanner" src/main.cpp; then
            echo ""
            echo "Current mode: I2C Scanner"
        else
            echo ""
            echo "Current mode: Application"
        fi
    fi
fi