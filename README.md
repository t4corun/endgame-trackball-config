### Firmware for PAW3395 

[Download latest UF2](https://nightly.link/efogtech/endgame-trackball-config/workflows/build/paw3395/firmware.zip)

### My Notes

Goals
- implement my keymap without changing Artem's board: `/boards/arm/efogtech_trackball_0`
- Keep changes to only the `/config` folder. There will be many undefines and behavior overrides. This also enables easier merging of Artem's future releases
- Implement parameterization when possible

