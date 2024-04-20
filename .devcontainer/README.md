# How to use Dev container

- Install Docker and the 'Devcontainer' extension in VSCode

- Clone the repository and open the folder in VSCode

- Use `Ctrl/Cmd + Shift + P` to open the command palette

- Select 'Dev containers: Rebuild and Reopen in Container' Option

- Wait for the container to finish setting up

- Type this command into the terminal `roslaunch pycram ik_and_description.launch` to start roscore

  - Note: Bullet world needs a display to be connected; the render mode needs to be set to direct when running on GitHub codespaces or on WindowOS; Otherwise, the kernel will crash
