# FFAC
This is a Demo reporsitory of our Fourier Feature Character Animation Controller which only contains the code are necessary for running the demo\
Full source code are not public yet
## Preview
![Previewvid](image/demo_thumbnail.gif)
## How to use
### Demo
If you just want to run the demo program without worring about the source code, simply run `Demo/Runtime_demo_by_YIFEI_CHEN.exe` on Windows.\
![Click](image/click.png)
### How to control
I strongly recommend you to have a controller ready on your computer, though you still can use keyboard to control the character, but the visual result is not as good as using controller. Theoretically any controller that can connect to Windows system should work, I tested with XBOX gamepad and it works fine. Different controller may have various button mapping and in that case the control keys could be different. 
![Gamepad](image/gamepad.JPG)

**Gamepad Control**
- Leftstick              - Moving Direction
- Rightsitck             - Camera Direction
- LT (HOLD) + Rightstick - Facing Direction
- RT (HOLD)              - Switch to Run

**Keyboard Control**
- W S A D (HOLD)                 - Moving Direction
- ALT (HOLD) + Leftclick (HOLD)  - Camera Direction
- X                              - Stop
- F                              - Switch to Run/Walk

## Build from Source
If you want to build the source code, you should follow the instructions below/
- a. Run `git submodule update --init` to install all required submodules
- b. (Windows) Download the pre-complied [GLFW](https://www.glfw.org/download) and [Libtorch](https://pytorch.org/) and place them in `3rd_party/glfw` and `3rd_party/libtorch`
![libtorch](image/libtorch.png| width=300)
- b. (Linux) Build the required package and place them in `3rd_party/glfw` and `3rd_party/libtorch`

Now your `3rd_party` directory should look like this 

.
├── _config.yml
├── _drafts
│   ├── begin-with-the-crazy-ideas.textile
│   └── on-simplicity-in-technology.markdown
├── _includes
│   ├── footer.html
│   └── header.html
├── _layouts
│   ├── default.html
│   └── post.html
