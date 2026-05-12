#!/bin/bash

# Argument check
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <right_hand_kernels> <left_hand_kernels>"
  echo "  <right_hand_kernels> : udev KERNELS of your USB-serial adapter of right hand (e.g., 1-2.2.3). You can check the KERNELS by 'udevadm info --name=<Device File of Right Hand> --attribute-walk'"
  echo "  <left_hand_kernels>  : udev KERNELS of your USB-serial adapter of left hand (e.g., 1-2.2.4). You can check the KERNELS by 'udevadm info --name=<Device File of Left Hand> --attribute-walk'"
  exit 1
fi

RHAND_KERNELS="$1"
LHAND_KERNELS="$2"

RULES_DIR="/etc/udev/rules.d"
TEMPLATE_PATH="$(dirname "$(readlink -f "$0")")/99-g1-inspire-unique-device-names.rules.template"
RULES_PATH="$RULES_DIR/99-g1-inspire-unique-device-names.rules"

echo ""
echo "This script generates udev rules to provide unique device names of Inspire hands on G1"
echo "Right hand KERNELS : $RHAND_KERNELS"
echo "Left hand KERNELS  : $LHAND_KERNELS"
echo "Output file        : $RULES_PATH"
echo ""

NEW_RULE=$(sed \
  -e "s|{{RHAND_KERNELS}}|$RHAND_KERNELS|" \
  -e "s|{{LHAND_KERNELS}}|$LHAND_KERNELS|" \
  "$TEMPLATE_PATH")

# Check existing file
if [ -f "$RULES_PATH" ]; then
  echo "The file $RULES_PATH already exists. Current contents:"
  echo "--------------------------------------------------"
  cat "$RULES_PATH"
  echo "--------------------------------------------------"
  echo "How would you like to proceed?"
  echo "  [o] Overwrite"
  echo "  [c] Cancel"
  read -p "Enter your choice [o/c]: " USER_CHOICE

  case "$USER_CHOICE" in
    [oO])
      echo "Overwriting $RULES_PATH..."
      echo "$NEW_RULE" | sudo tee "$RULES_PATH" > /dev/null
      ;;
    *)
      echo "Operation cancelled."
      exit 0
      ;;
  esac
else
  echo "Creating new rules file..."
  echo "$NEW_RULE" | sudo tee "$RULES_PATH" > /dev/null
fi

echo ""
echo "Restarting udev"
echo ""
sudo udevadm control --reload
sudo udevadm trigger

echo "udev rules installed"
