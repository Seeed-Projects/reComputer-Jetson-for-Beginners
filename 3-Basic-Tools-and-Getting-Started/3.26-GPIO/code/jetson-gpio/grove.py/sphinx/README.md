
Documenation Generation
=======================

We generate API documentation using [Sphinx](https://www.sphinx-doc.org) and
[AutoAPI](https://autoapi.readthedocs.io/index.html) on RPi Raspbian system.

Prepare on RPi
--------------
1. Clone this repo

   ```shell
   cd
   git clone https://github.com/Seeed-Studio/grove.py.git
   ```

2. Install this repo, refer to [installation](https://github.com/Seeed-Studio/grove.py/tree/master#installation)

3. Install tool packages

   ```shell
   sudo apt-get install python-sphinx python3-sphinx graphviz
   sudo pip  install autoapi sphinx-rtd-theme
   sudo pip3 install autoapi sphinx-rtd-theme
   ```

Document the python code
------------------------
   Please document the code follow the style [autoapi](https://autoapi.readthedocs.io/index.html#documenting-the-code),
   but [Google and NumPy style](https://www.sphinx-doc.org/en/master/usage/extensions/napoleon.html) is prefer.

   In the python docstring, [reStructuredTexat](https://github.com/seayxu/CheatSheet/blob/master/files/reStructuredText-Quick-Syntax.md)
   grammar could be used, will generate the contents except API part.


Begin generation
----------------
1. Export working copy to PYTHONPATH

   To get the lastest document specific to working copy (if you have some changes to the python source/docstring).
   ```shell
   cd ~/grove.py; export PYTHONPATH=`pwd`
   ```

   ***Without this setting, you could only get the document for the installed one of package grove.py***.

2. Run make, then the document htmls will be under folder ~/grove.py/docs/

   ```shell
   cd ~/grove.py
   make -C sphinx html
   ```

3. Open ~/grove.py/docs/index.html by browser to see result.

Pushlish the API document
-------------------------
   Push all contents under ~/grove.py/docs to branch
   [gh-pages](https://github.com/Seeed-Studio/grove.py/tree/gh-pages) of this repo.

