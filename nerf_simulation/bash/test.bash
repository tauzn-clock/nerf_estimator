# Get the full path to the directory containing the currently executing script
script_location="$(realpath "$(dirname "${BASH_SOURCE[0]}")")"

# Use the script location as a variable
echo "Script location: $script_location"
