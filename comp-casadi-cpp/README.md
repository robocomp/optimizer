# comp_casadi_python
Intro to component here

You need to install and compile casadi from source.
First install teh ipopt optimizer from the Ubuntu repo: sudo apt install  coinor-libipopt-dev
Then clone https://github.com/casadi/
mkdir build; cd build
cmake .. -DWITH_COMMON=ON -DWITH_IPOPT=ON
make
sudo make install

## Configuration parameters
As any other component, *comp_casadi_python* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <comp_casadi_python's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/comp_casadi_python config
```
