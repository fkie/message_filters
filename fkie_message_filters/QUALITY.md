This document is a declaration of software quality for the `fkie_message_filters` package, based on the guidelines in
[REP-2004](https://www.ros.org/reps/rep-2004.html).

# `fkie_message_filters` Quality Declaration

The package `fkie_message_filters` claims to be in the **Quality Level 2** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level N in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

The package follows the semantic versioning convention.

### Version Stability [1.ii]

The package has a stable version (>= 1.0.0).

### Public API Declaration [1.iii]

The public headers comprise the public API. These are the headers installed in `ìnclude/fkie_message_filters`, excluding `*_impl.hpp` headers. Also, all classes and objects residing in the `fkie_message_filters::helpers` namespace are considered implementation details.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

API stability will be guaranteed once the package has been released to a particular ROS distribution, with the exception of the ROS Rolling distribution.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

ABI stability will be guaranteed for all ROS distributions except for ROS Rolling.

## Change Control Process [2]

### Change Requests [2.i]

Third party changes except for those of the main author will only occur through change requests.

### Contributor Origin [2.ii]

All pull requests require a DCO to be accepted.

### Peer Review Policy [2.iii]

All pull requests will be reviewed by at least one reviewer before being accepted.

### Continuous Integration [2.iv]

All pull requests must pass the CI tests on all currently supported ROS distributions to be accepted.

### Documentation Policy [2.v]

The package has no documentation policy for pull requests.

## Documentation [3]

### Feature Documentation [3.i]

The package provides a C++ API which is fully documented.

### Public API Documentation [3.ii]

The Public API documentation is done via Doxygen comments. All Doxygen comments need to be kept up-to-date.

### License [3.iii]

### Copyright Statement [3.iv]

See [LICENSE](LICENSE).

### Lists and Peer Review [3.v.c]

The package is currently not part of any quality list.

## Testing [4]

### Feature Testing [4.i]

All features provides by the public API are subject to unit testing.

### Public API Testing [4.ii]

The public API is fully covered by unit tests.

### Coverage [4.iii]

The package is not subject to coverage testing.

### Performance [4.iv]

The package is not subject to performance testing.

### Linters and Static Analysis [4.v]

The package has a `clang-format` code style which is enforced for commits and pull requests. The package is not subject to static analysis.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

The package depends only on packages with equal or higher quality level.

### Optional Direct Runtime ROS Dependencies [5.ii]

### Direct Runtime non-ROS Dependency [5.iii]

The package has no direct runtime non-ROS dependencies.

## Platform Support [6]

The package supports all tier-1 platforms as defined in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).
Additionally, all platforms with C++17 compiler support will most likely work.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

Vulnerabilities will be disclosed in the [CHANGELOG.rst](CHANGELOG.rst) with a link to the relevant pull request.
Whether or not a bug is categorized as security vulnerability is at the discretion of the author.
