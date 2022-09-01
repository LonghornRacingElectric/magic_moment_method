
╭━╮╭━╮╱╱╱╱╱╱╱╱╱╭━╮╭━╮╱╱╱╱╱╱╱╱╱╱╱╭╮<br />
┃┃╰╯┃┃╱╱╱╱╱╱╱╱╱┃┃╰╯┃┃╱╱╱╱╱╱╱╱╱╱╭╯╰╮<br />
┃╭╮╭╮┣━━┳━━┳┳━━┫╭╮╭╮┣━━┳╮╭┳━━┳━╋╮╭╯<br />
┃┃┃┃┃┃╭╮┃╭╮┣┫╭━┫┃┃┃┃┃╭╮┃╰╯┃┃━┫╭╮┫┃<br />
┃┃┃┃┃┃╭╮┃╰╯┃┃╰━┫┃┃┃┃┃╰╯┃┃┃┃┃━┫┃┃┃╰╮<br />
╰╯╰╯╰┻╯╰┻━╮┣┻━━┻╯╰╯╰┻━━┻┻┻┻━━┻╯╰┻━╯<br />
╱╱╱╱╱╱╱╱╭━╯┃<br />
╱╱╱╱╱╱╱╱╰━━╯<br />
<br />MMM but expanded O>O.<br /><br />
Allows investigations into stability and performance of vehicles by sweeping possible steady state cornering vehicle states.<br />
Creates a performance envelope for vehicle cornering capabilities.


## -Installation and Setup-

    1) Git bash setup

        Install git bash https://git-scm.com/download/win
        This is for version control :) you may need to do some research in order to learn how this works...

    2) Install Python

        Install latest version of python: https://www.python.org/downloads/
        NOTE: when running installer, on the launching page of the installer, check the "Add Python X.XX to PATH" button to YES

    3) Clone REPO somewhere on your computer

        Go to the folder you want to install the REPO, right click, and click "Git BASH Here"

        Run the following command:

        git clone https://github.com/LonghornRacingElectric/magic_moment_method.git

        You will need to signin to your github account for this to work, and have your github account be added to the LHRe repo

    4) Install Python Packages

        To run the solver/engine/other scripts, installing the necessary python packages is required.

        (The following won't work if you didn't add Python to PATH. If you find the following isn't working, check the following to
        ensure python and pip are added to path: https://datatofish.com/add-python-to-windows-path/)
        
        To install packages, go to the REPO folder in file explorer, right click on the REPO folder, and click 'git bash here'.
        Once on the command line, run the following command:
        (The following line must be run from command line inside the MMM repo)
        
        pip install -r requirements.txt

        If this doesn't work, and you tried adding python to path, you can try the following commands:

        python -m pip install -r requirements.txt

        (If you use anaconda, try this:)

        conda install -r requirements.txt

    5) Install Visual Studio Code

        Install here: https://code.visualstudio.com/download

        I recommend adding jupyter notebooks plugin for easy data analysis 
        (explanation on how to do this here: https://towardsdatascience.com/installing-jupyter-notebook-support-in-visual-studio-code-91887d644c5d)


    6) Running the MMM Engine

        In your IDE or command line, run the magic_moment_man.py. 
        This will generate the data output from the sweep in this file: analysis/MMM.csv
        
        You can do data analysis on this .csv however you want -
        But I heavily recommend using the jupyter notebooks already setup already
        One such notebook is analysis/MMM_plotter.ipynb, which will open analysis/MMM.csv by default

    ^^^ For any issues with this setup & execution, please ask Kieran Cosgrove ^^^

░░░░░░░▄▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▄░░░░░░<br />
░░░░░░█░░▄▀▀▀▀▀▀▀▀▀▀▀▀▀▄░░█░░░░░<br />
░░░░░░█░█░▀░░░░░▀░░▀░░░░█░█░░░░░<br />
░░░░░░█░█░░░░░░░░▄▀▀▄░▀░█░█▄▀▀▄░<br />
█▀▀█▄░█░█░░▀░░░░░█░░░▀▄▄█▄▀░░░█░<br />
▀▄▄░▀██░█▄░▀░░░▄▄▀░░░░░░░░░░░░▀▄<br />
░░▀█▄▄█░█░░░░▄░░█░░░▄█░░░▄░▄█░░█<br />
░░░░░▀█░▀▄▀░░░░░█░██░▄░░▄░░▄░███<br />
░░░░░▄█▄░░▀▀▀▀▀▀▀▀▄░░▀▀▀▀▀▀▀░▄▀░<br />
░░░░█░░▄█▀█▀▀█▀▀▀▀▀▀█▀▀█▀█▀▀█░░░<br />
░░░░▀▀▀▀░░▀▀▀░░░░░░░░▀▀▀░░▀▀░░░░<br />

## -Folder Structure-

    documentation/
        explains what the Milliken Moment Method and Diagram is; and potential use cases
        Also please download RCVD from the google drive :)

    engine/ 
        used to solve for a specific state and set of car parameters
        further documentation on this process is up and coming

    vehicle_params/
        different sets of car parameters, these static parameters can be modified during sweeps

    magic_moment_man.py
        used to sweep different parameter & state sets, exports data to csv file
        running this script will update analysis/MMM.csv

    analysis/
        used to analyze data exported to csv file in jupyter notebooks for easy data analysis

    tests/
        used to verify engine functionality doesn't change on pull requests unless approved

    helpers/
        various helper methods and classes used by other modules
