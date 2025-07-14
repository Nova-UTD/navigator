# VSCode User Guide

Autocomplete/intellisense does not work by default due to packages being inaccessible to the container.

To connect VSCode to a container:
  1. Install the Dev Containers extension
  2. Open the VSCode command palette (hit Ctrl+Shift+P)
  3. Search for "Dev Containers: Reopen in Container"
  4. Click on this to launch

# For those using C++

If you install a package outside of navigator and need to access its headers, add the header location to `.vscode/c_cpp_properties.json`. Never add `/usr/include/**` because system includes are order dependent (see: this comment: https://github.com/microsoft/vscode-cpptools/issues/6114#issuecomment-692362735).