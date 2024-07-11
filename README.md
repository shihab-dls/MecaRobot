# Meca500 Flyscan IOC

This repository contains a prototype pythonSoftIOC. This IOC enables position control of Meca500 joints via a file of new line seperated joint sets.
Joint values must be comma seperated, and the file must be placed in the Trajectories directory.
The IOC can be run by executing:

```bash
> ./boot.sh
```

Three things are currently tracked:

- A PV update at every joint set during motion

- Target joint set at every desired joint set is printed to terminal

- String of the commands buffered into meca500, where Buffer Step is performed, is printed to terminal
