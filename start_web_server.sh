#!/bin/bash
# Script para iniciar el servidor web local en macOS / Linux
echo "⚡ Iniciando servidor local en http://localhost:8088 ..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/web_app"
python3 -m http.server 8088
