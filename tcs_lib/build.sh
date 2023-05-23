gcc -c -Wall -Werror -fpic -I inc src/i2c.c -o build/i2c.o
gcc -c -Wall -Werror -fpic -I inc src/tcs.c -o build/tcs.o
gcc -shared -o build/libtcs.so build/i2c.o build/tcs.o
sudo cp -r build/libtcs.so /usr/lib
sudo cp inc/* /usr/include
