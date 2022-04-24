# giraff_viewer
Intro to component here

### Installing Casadi from source

1. Install the following: ```sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends```
2. Install IPOPT: ```sudo apt-get install coinor-libipopt-dev```
4. Clone the casadi repo: ```git clone https://github.com/casadi/casadi.git```
5. Create a build directory for an out-of-source build:
```
cd casadi
cmake -DWITH_COMMON=ON -DWITH_IPOPT=ON .
make
sudo make install
```

## Configuration parameters
As any other component, *giraff_viewer* needs a configuration file to start. In
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
cd <giraff_viewer's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/giraff_viewer config
```
