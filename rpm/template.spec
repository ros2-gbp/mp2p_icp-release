%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/iron/.*$
%global __requires_exclude_from ^/opt/ros/iron/.*$

Name:           ros-iron-mp2p-icp
Version:        1.6.2
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS mp2p_icp package

License:        BSD
URL:            https://github.com/MOLAorg/mp2p_icp
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-iron-mola-common
Requires:       ros-iron-mrpt-libbase
Requires:       ros-iron-mrpt-libgui
Requires:       ros-iron-mrpt-libmaps
Requires:       ros-iron-mrpt-libobs
Requires:       ros-iron-mrpt-libposes
Requires:       ros-iron-mrpt-libtclap
Requires:       tbb-devel
Requires:       ros-iron-ros-workspace
BuildRequires:  cmake3
BuildRequires:  ros-iron-ament-cmake
BuildRequires:  ros-iron-mola-common
BuildRequires:  ros-iron-mrpt-libbase
BuildRequires:  ros-iron-mrpt-libgui
BuildRequires:  ros-iron-mrpt-libmaps
BuildRequires:  ros-iron-mrpt-libobs
BuildRequires:  ros-iron-mrpt-libposes
BuildRequires:  ros-iron-mrpt-libtclap
BuildRequires:  ros-iron-ros-environment
BuildRequires:  tbb-devel
BuildRequires:  ros-iron-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/iron" \
    -DCMAKE_PREFIX_PATH="/opt/ros/iron" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
%license COPYING
/opt/ros/iron

%changelog
* Sat Sep 14 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.6.2-1
- Autogenerated by Bloom

* Mon Sep 09 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.6.0-1
- Autogenerated by Bloom

* Tue Aug 27 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.5.5-1
- Autogenerated by Bloom

* Tue Aug 20 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.5.3-1
- Autogenerated by Bloom

* Wed Jul 24 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.5.2-1
- Autogenerated by Bloom

* Wed Jul 03 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.5.1-1
- Autogenerated by Bloom

* Fri Jun 21 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.5.0-1
- Autogenerated by Bloom

* Tue Jun 11 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.4.3-1
- Autogenerated by Bloom

* Tue May 28 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.4.2-1
- Autogenerated by Bloom

* Sun May 19 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.4.1-1
- Autogenerated by Bloom

* Mon May 06 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.4.0-1
- Autogenerated by Bloom

* Tue Apr 30 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.3.3-1
- Autogenerated by Bloom

* Mon Apr 22 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.3.2-1
- Autogenerated by Bloom

* Tue Apr 16 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.3.1-1
- Autogenerated by Bloom

* Sun Mar 10 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.3.0-1
- Autogenerated by Bloom

* Fri Feb 16 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.2.0-1
- Autogenerated by Bloom

* Wed Feb 07 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.1.1-1
- Autogenerated by Bloom

* Fri Jan 26 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.1.0-1
- Autogenerated by Bloom

* Sat Jan 20 2024 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 1.0.0-1
- Autogenerated by Bloom

* Wed Jun 14 2023 Jose-Luis Blanco-Claraco <joseluisblancoc@gmail.com> - 0.1.0-1
- Autogenerated by Bloom

