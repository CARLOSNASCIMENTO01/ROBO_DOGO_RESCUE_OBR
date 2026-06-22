#!/bin/bash

echo "?? Procurando Arduino..."

PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)

if [ -z "$PORT" ]; then
    echo "? Arduino n�o encontrado!"
    exit 1
fi

echo "? Porta encontrada: $PORT"

echo "?? Liberando serial..."
sudo killall python3 2>/dev/null

echo "?? Enviando c�digo..."
arduino-cli upload -p $PORT --fqbn arduino:avr:nano:cpu=atmega328old ~/OPENCV_LINE/codigo_arduino

echo "? Upload finalizado!"
