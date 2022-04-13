# comp-casadi-cpp

## Installation instructions

### Installing Casadi from source

1. Install the following: ```sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends```
2. Install IPOPT: ```sudo apt-get install coinor-libipopt-dev```
4. Clone the casadi repo: ```git clone https://github.com/casadi/casadi.git```
5. Create a build directory for an out-of-source build:
```
cd casadi
mkdir build
cd build
cmake -DWITH_COMMON=ON -DWITH_IPOPT=ON ..
make
sudo make install
```
### Installing the component
Once casadi is installed you can try installing the component using the following commands

```
cmake .
make
```

## Configuration parameters

As any other component, *comp_casadi_cpp* needs a configuration file to start. In
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
cd <comp_casadi_cpp's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/comp_casadi_cpp config
```

**NOTE:** In case of any problems with installation, please raise an issue.
