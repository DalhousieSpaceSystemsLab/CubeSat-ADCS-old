# Run this script before the UART test
# This scripts creates a PTY pair ~/COM1 and ~/COM2 and joins them
# Anything written to ~/COM1 can be read from ~/COM2 and vice versa
# The UART test reads to and writes from ~/COM1, this script uses ~/COM2

echo "Creating PTY ports..."
socat PTY,link=$HOME/com1,raw,echo=0 PTY,link=$HOME/com2,raw,echo=0 &
while true
do
rd=$(head ~/com2 -n1)
echo $rd
echo "{'received':'"$rd"'}" > ~/com2
sleep 0.5
done

