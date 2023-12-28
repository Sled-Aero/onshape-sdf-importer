Example usage: `python main.py thing /Users/liam/src/sled/PX4-SITL_gazebo/models/ f1acb5938f411c5a47db608c d20d9753a184585e00e13e9f 34860e26f5a79791793441a0`

### Fixing Shakiness
* Convert simulated fixed joints into static link groups
* Make prop link pose origin at the rotating center of mass
* Avoid inertia values that are too small along any axis

### Todo
* Make everything way cleaner
* Figure out how to actually calculate joint connection point (link frame pose now centered on com origin)
