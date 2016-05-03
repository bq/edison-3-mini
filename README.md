WHAT IS THIS?
=============

Linux Kernel source code for the devices:
* bq edison 3 mini


BUILD INSTRUCTIONS?
===================

Specific sources are separated by branches and each version is tagged with it's corresponding number. First, you should
clone the project:

        $ git clone https://github.com/bq/edison-3-mini.git

After it, choose the version you would like to build:

* Edison 3 Mini *

        $ cd edison3-mini/
        $ git checkout edison-3-mini

Finally, build the kernel:

        $ chmod +x Compile.sh
        $ ./Compile.sh
