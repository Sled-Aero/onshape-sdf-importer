Example usage: `python main.py scale_tiltwing /Users/liam/src/sled/PX4-SITL_gazebo/models/ f1acb5938f411c5a47db608c d20d9753a184585e00e13e9f ed0471d8793e2cd4e920b1dd`

### Fixing Shakiness
- [x] Convert simulated fixed joints into static link groups
- [x] Make prop link pose origin at the rotating center of mass
- [ ] Avoid inertia values that are too small along any axis

### Todo
* Make everything way cleaner
* Figure out how to actually calculate joint connection point (link frame pose now centered on com origin)
* Look into using https://pydantic-xml.readthedocs.io/en/latest/ for validation/generation (raw objects)
