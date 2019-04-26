echo "writing Arduino UNO"
echo "connect arduino via USB"

cd `dirname ${0}`/../platformio/ino01
sudo platformio run 
