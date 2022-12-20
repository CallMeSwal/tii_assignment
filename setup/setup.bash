set -e  # Exit on first error
APT_INSTALL='sudo apt-get install -y -qq'

install_dev_pkgs() {
  # General dev tools
  $APT_INSTALL \
    git \
    tree \
    htop \
    sshpass
}

install_cpp_pkgs() {
  # C / C++
  $APT_INSTALL \
    exuberant-ctags \
    automake \
    cmake \
    ccache \
    gcc \
    clang \
    clang-format \
    clang-tidy
}

install_python_pkgs() {
  # Python
  $APT_INSTALL \
    libpython3-dev \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-matplotlib
}

install_desktop_pkgs() {
  $APT_INSTALL \
    xterm \
    xinit \
    xbacklight \
    pavucontrol
}

git_config() {
  git config --global user.name "Vivek R";
  git config --global user.email "vivekrk44@gmail.com";
  git config --global push.default matching;
}

setup_configfiles() {
  # REMOVE OLD DOTFILES
  echo "remove old script files";
  rm -f "${HOME}/.bash_profile";
  rm -rf "${HOME}/.gitconfig";

  # SYMLINKS
  echo "symlinks script files";
  ln -fs "${PWD}/configs/bash_profile" "${HOME}/.bash_profile";
  ln -fs "${PWD}/configs/ros_profile" "${HOME}/.ros_profile";
  ln -fs "${PWD}/configs/gitconfig" "${HOME}/.gitconfig";
  echo "source ~/.bash_profile" >> "${HOME}/.bashrc";
  echo "source ~/.ros_profile" >> "${HOME}/.bashrc";
}


setup() {
  MODE=$1;
  git_config;
  install_dev_pkgs;
  install_cpp_pkgs;

  if [ "$MODE" == "full" ]; then
    install_python_pkgs;
    install_desktop_pkgs;
  fi

  setup_configfiles;
  echo "Done! :)"
}

print_usage() {
  echo "setup.bash [full | mini]"
}

# MAIN
if [ "$#" != "1" ]; then
  print_usage;
  exit;
fi
setup "${1}"
