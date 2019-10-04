#!/bin/bash

# Read command line input:
while [[ $# > 1 ]]; do
    key="$1"
    case $key in
        -h|--help)
            HELP="$2"
            shift # past argument
        ;;
        -f|--firmware)
            FIRMWARE="$2"
            shift # past argument
        ;;
        -n|--name)
            NAME="$2"
            shift # past argument
        ;;
        -v|--version)
            VERSION="$2"
            shift # past argument
        ;;
        -e|--environment)
            ENVIRONMENT="$2"
            shift # past argument
        ;;
        *)
            # unknown option
        ;;
    esac
    shift # past argument or value
done

function printOutput() {
    if hash msee 2>/dev/null; then
        echo "$1" | msee
    else
        echo "$1"
    fi
}

if [ ! -z "$HELP" ]; then
    output=$(cat <<EOM
# Help!
This script will to all after build steps you want to execute.

# tl;dr

    bash ${0}

# Usage

    bash ${0}
        [-h|--help 1]
        -n|--name <build name>
        -f|--firmware <firmware binary>
        -v|--version <version number>
        -e|--environment <build environment>

    IE:
        bash ${0} -f .pio/build/nodemcuv2_nystuen/firmware.bin -v 1001 -e nodemcuv2_nystuen
EOM
)
    printOutput "$output"
    exit 1;
fi

cp "${FIRMWARE}" "./versions/firmware-${ENVIRONMENT}-${VERSION}.bin"
aws s3 sync ./versions/ s3://litt.no-esp8266-fota/esp8266/fota/esp8266_power_meter/