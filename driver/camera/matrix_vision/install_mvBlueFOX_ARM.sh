#!/bin/bash
TARGET=undefined
TARGET_UNCAPITALIZED=undefined
DEF_DIRECTORY=/opt/mvIMPACT_Acquire
DEF_DATA_DIRECTORY=${MVIMPACT_ACQUIRE_DATA_DIR:-/opt/mvIMPACT_Acquire/data}
PRODUCT=mvBlueFOX
API=mvIMPACT_Acquire
TARNAME=mvBlueFOX
USE_DEFAULTS=NO
UNATTENDED_INSTALLATION=NO
MINIMAL_INSTALLATION=NO
APT_GET_EXTRA_PARAMS=
ARM_ARCHITECTURE="$(uname -m)"
OS_NAME="unknown"
OS_VERSION="unknown"
OS_CODENAME="unknown"
KERNEL_VERSION="unknown"

# Define a variable for the ErrorCount and WarningCount and an array for both to summarize the kind of issue
let ERROR_NUMBER=0
let WARNING_NUMBER=0

# Define variables for colorized bash output
# Foreground
red=`tput setaf 1`
yellow=`tput setaf 3`
green=`tput setaf 10`
blue=`tput setaf 12`
bold=`tput bold`
# Background
greyBG=`tput setaf 7`
reset=`tput sgr0`

# Define the users real name if possible, to prevent accidental mvIA root ownership if script is invoked with sudo
if [ "$(which logname)" == "" ] ; then
    USER=$(whoami)
else
    if [ "$(logname 2>&1 | grep -c logname:)" == "1" ] ; then
        USER=$(whoami)
    else
        USER=$(logname)
    fi
fi

# If user is root, then sudo shouldn't be used
if [ "$USER" == "root" ] ; then
        SUDO=
else
        SUDO=$(which sudo)
fi

function createSoftlink {
    if [ ! -e "$1/$2" ]; then
        echo "Error: File "$1/$2" does not exist, softlink cannot be created! "
        exit 1
    fi
    if [ -e "$1/$3" ]; then
        rm -rf "$1/$3" >/dev/null 2>&1
    fi
    if ! [ -L "$1/$3" ]; then
        ln -fs $2 "$1/$3" >/dev/null 2>&1
        if ! [ -L "$1/$3" ]; then
            $SUDO ln -fs $2 "$1/$3" >/dev/null 2>&1
            if ! [ -L "$1/$3" ]; then
                echo "Error: Could not create softlink $1/$3, even with sudo!"
                exit 1
            fi
        fi
    fi
}

# Print out ASCII-Art Logo.
clear;
echo ""
echo ""
echo ""
echo ""
echo "                                ===      ===     MMM~,    M                     "
echo "                                 ==+     ==      M   .M   M                     "
echo "                                 .==    .=+      M    M.  M   M    MM   ~MMM    "
echo "                                  ==+   ==.      MMMMM.   M   M    MM  M:   M   "
echo "             ..                    .== ,==       M   =M   M   M    MM +MMMMMMM  "
echo "   MMMM   DMMMMMM      MMMMMM       =====        M    MM  M   M    MM 7M        "
echo "   MMMM MMMMMMMMMMM :MMMMMMMMMM      ====        M    M+  M   MM  DMM  MM   ,   "
echo "   MMMMMMMMMMMMMMMMMMMMMMMMMMMMM                 IMM+''   M    .M:       =MI    "
echo "   MMMMMMM   .MMMMMMMM    MMMMMM                                                "
echo "   MMMMM.      MMMMMM      MMMMM                                                "
echo "   MMMMM       MMMMM       MMMMM                 MMMMMM    MMMMM    MM.   M     "
echo "   MMMMM       MMMMM       MMMMM                 M       MM    .MM  .M: .M      "
echo "   MMMMM       MMMMM       MMMMM                 M      .M       M~  .M~M       "
echo "   MMMMM       MMMMM       MMMMM                 MMMMMM MM       MD   +MM       "
echo "   MMMMM       MMMMM       MMMMM                 M       M       M    M MM      "
echo "   MMMMM       MMMMM       MMMMM                 M       MM     MM  :M. .M8     "
echo "   MMMMM       MMMMM       MMMMM                 M        .MMMMM    M     MD    "
echo "   MMMMM       MMMMM       MMMMM                                                "
echo ""
echo "================================================================================"
sleep 1

# Analyze the command line arguments and react accordingly
PATH_EXPECTED=NO
SHOW_HELP=NO
while [[ $# -gt 0 ]] ; do
  if [ "$1" == "-h" ] || [ "$1" == "--help" ] ; then
    SHOW_HELP=YES
    break
  elif [[ ( "$1" == "-u" || "$1" == "--unattended" ) && "$PATH_EXPECTED" == "NO" ]] ; then
    if [ "$MINIMAL_INSTALLATION" == "YES" ] ; then
      echo
      echo "WARNING: Unattended installation and minimal installation are mutually exclusive!"
      echo
      SHOW_HELP=YES
      break
    else
      UNATTENDED_INSTALLATION=YES
    fi
  elif [[ ( "$1" == "-m" || "$1" == "--minimal" ) && "$PATH_EXPECTED" == "NO" ]] ; then
    if [ "$UNATTENDED_INSTALLATION" == "YES" ] ; then
      echo
      echo "WARNING: Minimal installation and unattended installation are mutually exclusive!"
      echo
      SHOW_HELP=YES
      break
    else
      MINIMAL_INSTALLATION=YES
    fi
  elif [[ ( "$1" == "-p" || "$1" == "--path" ) && "$PATH_EXPECTED" == "NO" ]] ; then
    if [ "$2" == "" ] ; then
      echo
      echo "WARNING: Path option used with no defined path, will use: $DEF_DIRECTORY directory"
      echo
      SHOW_HELP=YES
      break
    else
      PATH_EXPECTED=YES
    fi
  elif [ "$PATH_EXPECTED" == "YES" ] ; then
    DEF_DIRECTORY=$1
    PATH_EXPECTED=NO
  else
    echo 'Please check your syntax and try again!'
    SHOW_HELP=YES
  fi
  shift
done

if [ "$SHOW_HELP" == "YES" ] ; then
  echo
  echo 'Installation script for the '$PRODUCT' driver.'
  echo
  echo "Default installation path: "$DEF_DIRECTORY
  echo "Usage:                     ./install_mvBlueFOX_ARM.sh [OPTION] ... "
  echo "Example:                   ./install_mvBlueFOX_ARM.sh -p /myPath -u"
  echo
  echo "Arguments:"
  echo "-h --help                  Display this help."
  echo "-p --path                  Set the directory where the files shall be installed."
  echo "-u --unattended            Unattended installation with default settings. By using"
  echo "                           this parameter you explicitly accept the EULA."
  echo "-m --minimal               Minimal installation. No tools or samples will be built, and"
  echo "                           no automatic configuration and/or optimizations will be done."
  echo "                           By using this parameter you explicitly accept the EULA."
  echo
  exit 1
fi

if [ "$UNATTENDED_INSTALLATION" == "YES" ] ; then
  echo
  echo "Unattended installation requested, no user interaction will be required and the"
  echo "default settings will be used."
  echo
  USE_DEFAULTS=YES
fi

if [ "$MINIMAL_INSTALLATION" == "YES" ] ; then
  echo
  echo "Minimal installation requested, no user interaction will be required, no tools or samples"
  echo "will be built and no automatic configurations or optimizations will be done."
  echo
  USE_DEFAULTS=YES
fi
# Get the source directory (the directory where the files for the installation are) and cd to it
# (The script file must be in the same directory as the source TGZ) !!!
if which dirname >/dev/null; then
    SCRIPTSOURCEDIR="$(dirname $(realpath $0))"
fi
if [ "$SCRIPTSOURCEDIR" != "$PWD" ]; then
   if [ "$SCRIPTSOURCEDIR" == "" ] || [ "$SCRIPTSOURCEDIR" == "." ]; then
      SCRIPTSOURCEDIR="$PWD"
   fi
   cd "$SCRIPTSOURCEDIR"
fi

# Set variables for GenICam and mvIMPACT_acquire for later use
if grep -q '/etc/ld.so.conf.d/' /etc/ld.so.conf; then
   ACQUIRE_LDSOCONF_FILE=/etc/ld.so.conf.d/acquire.conf
else
   ACQUIRE_LDSOCONF_FILE=/etc/ld.so.conf
fi

# Make sure the environment variables are set at the next boot as well
if grep -q '/etc/profile.d/' /etc/profile; then
   ACQUIRE_EXPORT_FILE=/etc/profile.d/acquire.sh
else
   ACQUIRE_EXPORT_FILE=/etc/profile
fi

# Get some details about the system
if which lsb_release >/dev/null 2>&1; then
  OS_NAME=$(lsb_release -is)
  OS_VERSION=$(lsb_release -rs)
  OS_CODENAME=$(lsb_release -cs)
  KERNEL_VERSION=$(uname -r)
fi

# Get driver name, version, file. In case of multiple *.tgz files in the folder select the newest version.
if [ "$( ls | grep -c 'mvBlueFOX.*\.tgz' )" != "0" ] ; then
  TARNAME=`ls mvBlueFOX*.tgz|tail -1 | sed -e s/\\.tgz//`
  if [ "$(echo $TARNAME | grep -c ARMhf)" != "0" ]; then
    TARGET="ARMhf"
    TARGET_UNCAPITALIZED="armhf"
  elif [ "$(echo $TARNAME | grep -c ARMsf)" != "0" ]; then
    TARGET="ARMsf"
    TARGET_UNCAPITALIZED="armsf"
  elif [ "$(echo $TARNAME | grep -c ARM64)" != "0" ]; then
    TARGET="ARM64"
    TARGET_UNCAPITALIZED="arm64"
  else
    echo "Error: Could not determine target architecture from filename."
    echo "In case the file been renamed, please revert to original name."
    echo "Terminating this installation script..."
    echo
    exit 1
  fi 
  TARFILE=`ls mvBlueFOX*.tgz|tail -1`
  VERSION=`ls mvBlueFOX*.tgz|tail -1 | sed -e s/\\mvBlueFOX// | sed -e s/\\-"$TARGET"_gnu.*-// | sed -e s/\\.tgz//`
  ACT=$API-$TARGET-$VERSION
  ACT2=$ACT
fi

YES_NO=""
# Ask whether to use the defaults or proceed with an interactive installation
if [ "$UNATTENDED_INSTALLATION" == "NO" ] && [ "$MINIMAL_INSTALLATION" == "NO" ] ; then
  echo
  echo "Would you like this installation to run in unattended mode?"
  echo "Using this mode you explicitly agree to the EULA(End User License Agreement)!"
  echo "No user interaction will be required, and the default settings will be used!"
  echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
  read YES_NO
else
  YES_NO=""
fi
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
  USE_DEFAULTS=NO
else
  USE_DEFAULTS=YES
fi

YES_NO=""
# Here we will ask the user if we shall start the installation process
echo
echo   "-----------------------------------------------------------------------------------"
echo   "${bold}Host System:${reset}"
echo   "-----------------------------------------------------------------------------------"
echo
echo   "${bold}OS:                             ${reset}"$OS_NAME
echo   "${bold}OS Version:                     ${reset}"$OS_VERSION
echo   "${bold}OS Codename:                    ${reset}"$OS_CODENAME
echo   "${bold}Kernel:                         ${reset}"$KERNEL_VERSION
echo   "${bold}Platform:                       ${reset}"$TARGET
echo
echo "-----------------------------------------------------------------------------------"
echo "${bold}Configuration:${reset}"
echo "-----------------------------------------------------------------------------------"
echo
echo "${bold}Installation for user:            ${reset}"$USER
echo "${bold}Installation directory:           ${reset}"$DEF_DIRECTORY
echo "${bold}Data directory:                   ${reset}"$DEF_DATA_DIRECTORY
echo "${bold}Source directory:                 ${reset}"$(echo $SCRIPTSOURCEDIR | sed -e 's/\/\.//')
echo "${bold}Version:                          ${reset}"$VERSION
echo "${bold}TAR-File:                         ${reset}"$TARFILE
echo
echo "${bold}ldconfig:"
echo "${bold}mvIMPACT_acquire:                 ${reset}"$ACQUIRE_LDSOCONF_FILE
echo
echo "${bold}Exports:"
echo "${bold}mvIMPACT_acquire:                 ${reset}"$ACQUIRE_EXPORT_FILE
echo
echo "-----------------------------------------------------------------------------------"
echo
echo "Do you want to continue (default is 'yes')?"
echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
if [ "$USE_DEFAULTS" == "NO" ] ; then
  read YES_NO
else
  YES_NO=""
fi

echo
# If the user is choosing no, we will abort the installation, else we will start the process.
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
  echo "Quit!"
  exit
fi

# End User License Agreement
YES_NO="r"
while [ "$YES_NO" == "r" ] || [ "$YES_NO" == "R" ]
do
  echo
  echo "Do you accept the End User License Agreement (default is 'yes')?"
  echo "Hit 'n' + <Enter> for 'no', 'r' + <Enter> to read the EULA or "
  echo "just <Enter> for 'yes'."
  if [ "$USE_DEFAULTS" == "NO" ] ; then
    read YES_NO
    if [ "$YES_NO" == "r" ] || [ "$YES_NO" == "R" ] ; then
    if [ "x$(which more)" != "x" ] ; then
      EULA_SHOW_COMMAND="more -d"
    else
      EULA_SHOW_COMMAND="cat"
    fi
    tar -xzf $TARFILE -C /tmp mvIMPACT_acquire-$VERSION.tar && tar -xf /tmp/mvIMPACT_acquire-$VERSION.tar -C /tmp mvIMPACT_acquire-$VERSION/doc/EULA.txt --strip-components=2 && rm /tmp/mvIMPACT_acquire-$VERSION.tar && $EULA_SHOW_COMMAND /tmp/EULA.txt && rm /tmp/EULA.txt && sleep 1
    # clear the stdin buffer in case user spammed the Enter key
    while read -r -t 0; do read -r; done
    fi
  else
    YES_NO=""
  fi
done

# If the user is choosing no, we will abort the installation, else we continue.
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
  echo "Quit!"
  exit
fi

echo
echo   "-----------------------------------------------------------------------------------"
echo   "${bold}BY INSTALLING THIS SOFTWARE YOU HAVE AGREED TO THE EULA(END USER LICENSE AGREEMENT)${reset}"
echo   "-----------------------------------------------------------------------------------"
echo

# First of all ask whether to dispose of the old mvIMPACT Acquire installation
if [ "$MVIMPACT_ACQUIRE_DIR" != "" ]; then
  echo "Existing installation detected at: $MVIMPACT_ACQUIRE_DIR"
  echo "Do you want to keep this installation (default is 'yes')?"
  echo "If you select no, mvIMPACT Acquire will be removed for ALL installed products!"
  echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
  if [ "$USE_DEFAULTS" == "NO" ] ; then
    read YES_NO
  else
    YES_NO=""
  fi
  echo
  if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
    $SUDO rm -f /usr/bin/mvDeviceConfigure >/dev/null 2>&1
    $SUDO rm -f /usr/bin/mvIPConfigure >/dev/null 2>&1
    $SUDO rm -f /usr/bin/wxPropView >/dev/null 2>&1
    $SUDO rm -f /etc/ld.so.conf.d/acquire.conf >/dev/null 2>&1
    $SUDO rm -f /etc/ld.so.conf.d/genicam.conf >/dev/null 2>&1
    $SUDO rm -f /etc/profile.d/acquire.sh >/dev/null 2>&1
    $SUDO rm -f /etc/profile.d/genicam.sh >/dev/null 2>&1
    $SUDO rm -f /etc/udev/rules.d/51-mvbf.rules >/dev/null 2>&1
    $SUDO rm -f /etc/udev/rules.d/52-U3V.rules >/dev/null 2>&1
    $SUDO rm -f /etc/udev/rules.d/52-mvbf3.rules >/dev/null 2>&1
    $SUDO rm -f /etc/sysctl.d/62-buffers-performance.conf >/dev/null 2>&1
    $SUDO rm -f /etc/security/limits.d/acquire.conf >/dev/null 2>&1
    $SUDO rm -rf /etc/matrix-vision >/dev/null >/dev/null 2>&1
    $SUDO rm -rf $MVIMPACT_ACQUIRE_DIR >/dev/null 2>&1
    if [ $? == 0 ]; then
      echo "Previous mvIMPACT Acquire Installation ($MVIMPACT_ACQUIRE_DIR) removed successfully!"
    else
      echo "Error removing previous mvIMPACT Acquire Installation ($MVIMPACT_ACQUIRE_DIR)!"
      echo "$?"
    fi
  else
    echo "Previous mvIMPACT Acquire Installation ($MVIMPACT_ACQUIRE_DIR) NOT removed!"
  fi
fi

# Create the *.conf files if the system is supporting ld.so.conf.d
if grep -q '/etc/ld.so.conf.d/' /etc/ld.so.conf; then
  $SUDO rm -f $ACQUIRE_LDSOCONF_FILE; $SUDO touch $ACQUIRE_LDSOCONF_FILE
fi

# Create the export files if the system is supporting profile.d
if grep -q '/etc/profile.d/' /etc/profile; then
  $SUDO rm -f $ACQUIRE_EXPORT_FILE; $SUDO touch $ACQUIRE_EXPORT_FILE
fi

# Check if the destination directory exist, else create it
if ! [ -d $DEF_DIRECTORY ]; then
  # the destination directory does not yet exist
  # first try to create it as a normal user
  mkdir -p $DEF_DIRECTORY >/dev/null 2>&1
  if ! [ -d $DEF_DIRECTORY ]; then
    # that didn't work
    # now try it as superuser
    $SUDO mkdir -p $DEF_DIRECTORY
  fi
  if ! [ -d $DEF_DIRECTORY  ]; then
    echo 'ERROR: Could not create target directory' $DEF_DIRECTORY '.'
    echo 'Problem:'$?
    echo 'Maybe you specified a partition that was mounted read only?'
    echo
    exit
  fi
else
  echo 'Installation directory already exists.'
fi

# in case the directory already existed BUT it belongs to other user
$SUDO chown -R $USER: $DEF_DIRECTORY

# Check the actual tarfile
if ! [ -r $TARFILE ]; then
  echo 'ERROR: could not read' $TARFILE.
  echo
  exit
fi

# needed at compile time (used during development, but not shipped with the final program)
ACT=$API-$VERSION.tar

# Now unpack the tar-file into /tmp
cd /tmp
rm -rf mvIMPACT_Acquire-ARM*
tar xfz "$SCRIPTSOURCEDIR/$TARFILE"

# Now check if we can unpack the tar file with the device independent stuff
# this is entirely optional
if [ -r /tmp/$ACT2 ]; then
   cd /tmp
   #tar xvf /tmp/$ACT
   cp -r $ACT2/* $DEF_DIRECTORY
else
  echo
  echo "ERROR: Could not read: /tmp/"$ACT2
  exit
fi

#Set the necessary exports and library paths
cd $DEF_DIRECTORY
if grep -q 'MVIMPACT_ACQUIRE_DIR=' $ACQUIRE_EXPORT_FILE; then
   echo 'MVIMPACT_ACQUIRE_DIR already defined in' $ACQUIRE_EXPORT_FILE.
else
   $SUDO sh -c "echo 'export MVIMPACT_ACQUIRE_DIR=$DEF_DIRECTORY' >> $ACQUIRE_EXPORT_FILE"
fi

if grep -q "$DEF_DIRECTORY/lib/$TARGET_UNCAPITALIZED" $ACQUIRE_LDSOCONF_FILE; then
   echo "$DEF_DIRECTORY/lib/$TARGET_UNCAPITALIZED already defined in" $ACQUIRE_LDSOCONF_FILE.
else
   $SUDO sh -c "echo '$DEF_DIRECTORY/lib/$TARGET_UNCAPITALIZED' >> $ACQUIRE_LDSOCONF_FILE"
fi
if grep -q "$DEF_DIRECTORY/Toolkits/expat/bin/$TARGET_UNCAPITALIZED/lib" $ACQUIRE_LDSOCONF_FILE; then
   echo "$DEF_DIRECTORY/Toolkits/expat/bin/$TARGET_UNCAPITALIZED/lib already defined in" $ACQUIRE_LDSOCONF_FILE.
else
   $SUDO sh -c "echo '$DEF_DIRECTORY/Toolkits/expat/bin/$TARGET_UNCAPITALIZED/lib' >> $ACQUIRE_LDSOCONF_FILE"
fi

# This variable must be exported, or else wxPropView-related make problems can arise
export MVIMPACT_ACQUIRE_DIR=$DEF_DIRECTORY

# Update the library cache with ldconfig
$SUDO /sbin/ldconfig

# Clean up /tmp
rm -rf /tmp/$ACT2 /tmp/$API-$VERSION

#create softlinks for the Toolkits libraries
createSoftlink $DEF_DIRECTORY/Toolkits/expat/bin/$TARGET_UNCAPITALIZED/lib $(ls $DEF_DIRECTORY/Toolkits/expat/bin/$TARGET_UNCAPITALIZED/lib | grep libexpat\.so\..*\..*\. ) libexpat.so.1
createSoftlink $DEF_DIRECTORY/Toolkits/expat/bin/$TARGET_UNCAPITALIZED/lib libexpat.so.1 libexpat.so
createSoftlink $DEF_DIRECTORY/Toolkits/FreeImage3160/bin/Release/FreeImage/$TARGET_UNCAPITALIZED $(ls $DEF_DIRECTORY/Toolkits/FreeImage3160/bin/Release/FreeImage/$TARGET_UNCAPITALIZED | grep libfreeimage-3\..*\.so ) libfreeimage.so.3
createSoftlink $DEF_DIRECTORY/Toolkits/FreeImage3160/bin/Release/FreeImage/$TARGET_UNCAPITALIZED libfreeimage.so.3 libfreeimage.so

#An important distinction has to be made here between 32bit and 64bit ARM systems
if [ $TARGET != "ARM64" ]; then
# Since the native make target for 32bit ARM architectures can have many different values 
# (eg. armv7l, arm7ahf etc. ) and will not be 'armhf' or 'armsf', a softlink has to be created,
# otherwise the mv apps and tools will not be able to be linked with the mv libraries.
    createSoftlink $DEF_DIRECTORY/lib $TARGET_UNCAPITALIZED $ARM_ARCHITECTURE
    createSoftlink $DEF_DIRECTORY/Toolkits/expat/bin $TARGET_UNCAPITALIZED $ARM_ARCHITECTURE
    createSoftlink $DEF_DIRECTORY/Toolkits/FreeImage3160/bin/Release/FreeImage $TARGET_UNCAPITALIZED $ARM_ARCHITECTURE
else
# In case of 64bit ARM architectures, they always report 'aarch64' back, which makes our lives 
# considerably easier. In this case we do not need to create softlinks, but the $ARM_ARCHITECTURE
# needs to be overwritten so that the mvApps softlinks in /usr/bin will point to the correct path.
    ARM_ARCHITECTURE=$TARGET_UNCAPITALIZED
fi

# Update the library cache with ldconfig
$SUDO /sbin/ldconfig

if [ "$MINIMAL_INSTALLATION" == "NO" ] ; then
  # apt-get extra parameters
  if [ "$USE_DEFAULTS" == "YES" ] ; then
    APT_GET_EXTRA_PARAMS=" -y --force-yes"
  fi
  
  
  # Ask whether the samples should be built natively
  echo
  echo "Do you want the sample applications to be built (default is 'yes')?"
  echo "A native g++ compiler has to be present on the system!"
  echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
  if [ "$USE_DEFAULTS" == "NO" ] ; then
    read YES_NO
  else
    YES_NO="N"
  fi
  echo
  if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
    echo 'The tools and samples were not built.'
    echo 'To build them yourself, type:'
    echo '  cd '$DEF_DIRECTORY
    echo '  make native'
    echo '  sudo /sbin/ldconfig'
  else
    if [ "$(which g++)" != "" ]; then
      echo "Do you want the GUI tools to be built (default is 'yes')?"
      echo "This requires wxWidgets libraries to be present on your system."
      echo "If they are missing, an attempt will be made to download them."
      echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
      if [ "$USE_DEFAULTS" == "NO" ] ; then
        read YES_NO
      else
        YES_NO=""
      fi
      echo
      if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
        # remove GUI apps sources since they are not needed
        rm -rf $DEF_DIRECTORY/apps/mv*
        rm -rf $DEF_DIRECTORY/apps/Common/FirmwareUpdate_mvHYPERION
      else
        # check if wxwidgets are present else download them
        if [ "$(wx-config --release 2>&1 | grep -c "^3.")" != "1" ]; then
          if [ "x$(which apt-get)" != "x" ]; then
            echo
            echo "Updating file lists from repositories..."
            echo
            $SUDO apt-get update
            echo
            echo "Downloading and installing wxWidgets via apt-get..."
            VERSION="$(cut -d'.' -f1 <<< $OS_VERSION)"
            if [ "$OS_NAME" == "Ubuntu" ] && [ "$VERSION" -ge "20" ]; then
               $SUDO apt-get $APT_GET_EXTRA_PARAMS -q install libwxgtk3.0-gtk3-* libwxgtk3.0-gtk3-dev libwxbase3.0-0* libwxbase3.0-dev wx3.0-headers build-essential libgtk2.0-dev
            else
               $SUDO apt-get $APT_GET_EXTRA_PARAMS -q install libwxgtk3.0-dev libwxbase3.0-0* libwxbase3.0-dev libwxgtk3.0-0* wx3.0-headers build-essential libgtk2.0-dev
            fi
            echo
            if [ $? == 0 ] && [ "x$(which wx-config)" != "x" ]; then
              echo "Necessary wxWidgets libraries installed successfully!"
              echo
            else
              echo "wxWidgets libraries could not automatically download and install on this system!"
              echo "Please either install wxWidgets libraries manually and re-run this installer script,"
              echo "or re-run this script and choose not to build the wxWidgets GUI Tools altogether!"
              echo
              exit 1
            fi
          else
            echo
            echo "Could not download wxWidgets, apt-get is missing!"
            echo
            echo "wxWidgets libraries could not automatically download and install on this system!"
            echo "Please either install wxWidgets libraries manually and re-run this installer script,"
            echo "or re-run this script and choose not to build the wxWidgets GUI Tools altogether!"
            echo
            exit 1
          fi
        fi
        cd $DEF_DIRECTORY
        $SUDO /sbin/ldconfig
      fi
      # build all apps and samples.
      echo "Building samples and/or tools..."
      make native
      if [ $? -ne 0 ]; then
        let WARNING_NUMBER=WARNING_NUMBER+1
      fi
      
      # Shall the MV tools be linked in /usr/bin?
      echo "Do you want to set a link to /usr/bin for wxPropView and mvDeviceConfigure (default is 'yes')?"
      echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
      if [ "$USE_DEFAULTS" == "NO" ] ; then
        read YES_NO
      else
        YES_NO=""
      fi
      if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
        echo "Will not set any new link to /usr/bin."
      else
        if [ -r /usr/bin ]; then
          # Set wxPropView
          if [ -r $DEF_DIRECTORY/apps/mvPropView/$ARM_ARCHITECTURE/wxPropView ]; then
            $SUDO rm -f /usr/bin/wxPropView
            $SUDO ln -s $DEF_DIRECTORY/apps/mvPropView/$ARM_ARCHITECTURE/wxPropView /usr/bin/wxPropView
          fi
          # Set mvDeviceConfigure
          if [ -r $DEF_DIRECTORY/apps/mvDeviceConfigure/$ARM_ARCHITECTURE/mvDeviceConfigure ]; then
            $SUDO rm -f /usr/bin/mvDeviceConfigure
            $SUDO ln -s $DEF_DIRECTORY/apps/mvDeviceConfigure/$ARM_ARCHITECTURE/mvDeviceConfigure /usr/bin/mvDeviceConfigure
          fi
        fi
      fi

      # Should wxPropView check weekly for updates?
        echo "Do you want wxPropView to check for updates weekly(default is 'yes')?"
        echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
        if [ "$USE_DEFAULTS" == "NO" ] ; then
          read YES_NO
        else
          YES_NO=""
        fi
        if [ ! -e ~/.wxPropView ]; then
          touch ~/.wxPropView
        fi
        if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
          if [ "$(grep -c AutoCheckForUpdatesWeekly ~/.wxPropView)" -ne "0" ]; then
            Tweakline=$(( $( grep -n "AutoCheckForUpdatesWeekly" ~/.wxPropView | cut -d: -f1) )) && sed -i "$Tweakline s/.*/AutoCheckForUpdatesWeekly=0/" ~/.wxPropView
          else
            echo "AutoCheckForUpdatesWeekly=0" >> ~/.wxPropView
          fi
        else
          if [ "$(grep -c AutoCheckForUpdatesWeekly ~/.wxPropView)" -ne "0" ]; then
            Tweakline=$(( $( grep -n "AutoCheckForUpdatesWeekly" ~/.wxPropView | cut -d: -f1) )) && sed -i "$Tweakline s/.*/AutoCheckForUpdatesWeekly=1/" ~/.wxPropView
          else
            echo "[MainFrame/Help]" >> ~/.wxPropView
            echo "AutoCheckForUpdatesWeekly=1" >> ~/.wxPropView
          fi
        fi
    else
      echo "Sample applications and/or GUI tools cannot be built, as the system is missing a g++ compiler!"
    fi
  fi
fi

# Update the library cache again.
$SUDO /sbin/ldconfig

# create the logs directory and set MVIMPACT_ACQUIRE_DATA_DIR.
if ! [ -d $DEF_DATA_DIRECTORY/logs ]; then
  mkdir -p $DEF_DATA_DIRECTORY/logs >/dev/null 2>&1
  if ! [ -d $DEF_DATA_DIRECTORY/logs ]; then
      # that didn't work, now try it as superuser
      $SUDO mkdir -p $DEF_DATA_DIRECTORY/logs >/dev/null 2>&1
  fi
fi

if [ -d $DEF_DATA_DIRECTORY/logs ]; then
  mv $DEF_DIRECTORY/apps/mvDebugFlags.mvd $DEF_DATA_DIRECTORY/logs >/dev/null 2>&1
  if ! [ -r $DEF_DATA_DIRECTORY/logs/mvDebugFlags.mvd ]; then
    $SUDO mv $DEF_DIRECTORY/apps/mvDebugFlags.mvd $DEF_DATA_DIRECTORY/logs >/dev/null 2>&1
  fi
  if grep -q 'MVIMPACT_ACQUIRE_DATA_DIR=' $ACQUIRE_EXPORT_FILE; then
    echo 'MVIMPACT_ACQUIRE_DATA_DIR already defined in' $ACQUIRE_EXPORT_FILE.
  else
    $SUDO sh -c "echo 'export MVIMPACT_ACQUIRE_DATA_DIR=$DEF_DATA_DIRECTORY' >> $ACQUIRE_EXPORT_FILE"
  fi
else
  echo "ERROR: Could not create " $DEF_DATA_DIRECTORY/logs " directory."
  echo 'Problem:'$?
  echo 'Maybe you specified a partition that was mounted read only?'
  echo
  exit
fi

# copy the udev rules file for the mvBlueFOX on the target system
echo
echo "Do you want to copy 51-mvbf.rules to /etc/udev/rules.d for non-root user support (default is 'yes')?"
echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
if [ "$USE_DEFAULTS" == "NO" ] ; then
  read YES_NO
else
  YES_NO=""
fi
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
   echo
   echo 'To grant non-root user support,'
   echo 'copy 51-mvbf.rules the file to /etc/udev/rules.d'
   echo
else
   $SUDO cp -f $DEF_DIRECTORY/Scripts/51-mvbf.rules /etc/udev/rules.d
fi

# check if plugdev group exists and the user is member of it
if [ "$(grep -c ^plugdev: /etc/group )" == "0" ]; then
  echo "Group 'plugdev' doesn't exist, this is necessary to use USB devices as a normal user,"
  echo "do you want to create it and add current user to 'plugdev' (default is 'yes')?"
  echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
  if [ "$USE_DEFAULTS" == "NO" ] ; then
    read YES_NO
  else
    YES_NO=""
  fi
  if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
    echo
    echo "'plugdev' will not be created and you can't run the device as non-root user!"
    echo "If you want non-root users support, you will need to create 'plugdev'"
    echo "and add the users to this group."
    let WARNING_NUMBER=WARNING_NUMBER+1
 else
    $SUDO /usr/sbin/groupadd -g 46 plugdev
    $SUDO /usr/sbin/usermod -a -G plugdev $USER
    echo "Group 'plugdev' created and user '"$USER"' added to it."
  fi
else
  if [ "$( groups | grep -c plugdev )" == "0" ]; then
    echo "Group 'plugdev' exists, however user '"$USER"' is not a member, which is necessary to"
    echo "use USB devices. Do you want to add  user '"$USER"' to 'plugdev' (default is 'yes')?"
    echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
    if [ "$USE_DEFAULTS" == "NO" ] ; then
      read YES_NO
    else
      YES_NO=""
    fi
    if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
      echo
      echo "If you want to use USB devices you have to manually add user '"$USER"' to the plugdev group."
      let WARNING_NUMBER=WARNING_NUMBER+1
    else
      $SUDO /usr/sbin/usermod -a -G plugdev $USER
      echo "User '"$USER"' added to 'plugdev' group."
    fi
  fi
fi

# make sure the complete mvIA-tree and the data folder belongs to the user
$SUDO chown -R $USER: $DEF_DIRECTORY
$SUDO chown -R $USER: $DEF_DATA_DIRECTORY

# Configure the /etc/security/limits.d/acquire.conf file to be able to set thread priorities
if [ -d /etc/security/limits.d ]; then
  if [[ ! -f /etc/security/limits.d/acquire.conf || "$(grep -c '@plugdev            -       nice            -20' /etc/security/limits.d/acquire.conf )" == "0" ]] ; then
    echo '@plugdev            -       nice            -20' | sudo tee -a /etc/security/limits.d/acquire.conf >/dev/null
  fi
  if [ "$(grep -c '@plugdev            -       rtprio          99' /etc/security/limits.d/acquire.conf )" == "0" ] ; then
    echo '@plugdev            -       rtprio          99' | sudo tee -a /etc/security/limits.d/acquire.conf >/dev/null
  fi
else
  echo 'INFO: Directory /etc/security/limits.d is missing, mvIMPACT Acquire will not'
  echo 'be able to set thread priorities correctly. Incomplete frames may occur!!!'
  let WARNING_NUMBER=WARNING_NUMBER+1
fi

# remove all example application sources in case of minimal installation 
if [ "$MINIMAL_INSTALLATION" == "YES" ] ; then
  $SUDO rm -rf $DEF_DIRECTORY/apps >/dev/null 2>&1
fi

echo
if [ "$ERROR_NUMBER" == 0 ] && [ "$WARNING_NUMBER" == 0 ]; then
    echo "-----------------------------------------------------------------------------------"
    echo "${green}${bold}                           Installation successful!${reset}         "
    echo "-----------------------------------------------------------------------------------"
elif [ "$ERROR_NUMBER" == 0 ] && [ "$WARNING_NUMBER" != 0 ]; then
    echo "-----------------------------------------------------------------------------------"
    echo "${yellow}${bold}                           Installation successful!${reset}        "
    echo "-----------------------------------------------------------------------------------"
    echo "                                                                                   "
    echo "  Some warnings have been issued during the installation. Typically the driver     "
    echo "  will work, but some functionalities are missing e.g. some sample applications    "
    echo "  which could not be built because of missing dependencies.                        "
    echo "                                                                                   "
    echo "  Please refer to the output of the script for further details.                    "
    echo "-----------------------------------------------------------------------------------"
else
    echo "-----------------------------------------------------------------------------------"
    echo "${red}${bold}                        Installation NOT successful!${reset}          "
    echo "-----------------------------------------------------------------------------------"
    echo "                                                                                   "
    echo "  Please provide the full output of this installation script to the MATRIX VISION  "
    echo "  support department if the error messages shown during the installation procedure "
    echo "  don't help you to get the driver package installed correctly!                    "
    echo "-----------------------------------------------------------------------------------"
fi
echo
echo "Do you want to reboot now (default is 'yes')?"
echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
if [ "$USE_DEFAULTS" == "NO" ] ; then
  read YES_NO
else
  YES_NO="n"
fi
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
   echo "You need to reboot manually to complete the installation."
else
   $SUDO shutdown -r now
fi
