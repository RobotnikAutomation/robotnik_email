#!/bin/bash

# Check if the recipient email is provided as an argument
if [ -z "$1" ]; then
    echo "Usage: $0 <recipient_email> [namespace]"
    exit 1
fi
# Check if the NODE_NAMESPACE is provided as an argument
if [ -n "$2" ]; then
    NODE_NAMESPACE="$2"
else
    NODE_NAMESPACE="/smtp_manager"
fi
RECIPIENT_EMAIL=$1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run the rosservice call command
rosservice call $NODE_NAMESPACE/send_email "recipients: ['$RECIPIENT_EMAIL']
status: {id: 0, description: 'Hello Test!', type: '', message: 'This is a test email'}
uuid: ''
datetime: ''
files_to_upload: ['$SCRIPT_DIR/imgs/ok.png']"
