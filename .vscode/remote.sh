rm .vscode/connect-ok
proxychains -q ssh $1@$2 "killall gdbserver"
echo "[SingleFlight-DEBUG] start scp EXE to Target ..."
proxychains -q scp build/flight $1@$2:/usr/bin
# proxychains -q scp APMconfig.json $1@$2:/etc
echo "[SingleFlight-DEBUG] lanuch SingleFlight Directly ..."
touch .vscode/connect-ok
proxychains -q ssh $1@$2 "/usr/bin/gdbserver :9590 /usr/bin/flight $3"
