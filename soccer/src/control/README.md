# UTRA Soccer Control Team
Bipedal control! Based on [UT Austin Villa](https://github.com/LARG/utaustinvilla3d).

### Building
Make sure that you have ->"rcssserver3d-dev"<- (the -dev part is important) installed. [Instructions here](http://simspark.sourceforge.net/wiki/index.php/Installation_on_Linux).

### Compiling
using cmake for compiling codes

```
cmake .
make
```

### Running
Once you've built successfully, start the simulation server in one terminal window using:

```bash
rcsoccersim3d
```

The soccer field should come up. Then start an agent in another window with (from inside of the build directory)

```bash
./test.sh
```



