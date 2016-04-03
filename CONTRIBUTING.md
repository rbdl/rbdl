# Coding style for RBDL

This documents gives an overview of the coding style used in RBDL and also
the general goals of RBDL.

If you are considering contributing to this library please read the whole
document.

## General Purpose of RBDL

RBDL implements large parts of the algorithms and methods described in
Featherstone's book Rigid Body Dynamics Algorithms. One of the main goals
of this library is to serve as an efficient implementation for the methods
described in Featherstone's book and to a lesser extent general multibody
dynamics and kinematics computations. A person who is familiar with
Featherstone's book should have an easy time to understand the code and
therefore the variable naming conventions used in the book should when
possible used in the code, e.g.: 

 - the joint space inertia matrix is denoted with H
 - the coriolis forces are denoted with C

The algorithmic parts of RBDL's code try to follow the algorithmic or
mathematical notations instead of wrapping algorithms in elegant
programming patterns.

### Aims and Non-Aims of RBDL

This is what RBDL aims to be:

* RBDL aims to be lean 
* RBDL aims to be easily integrated into other projects
* RBDL aims to be suitable as a foundation for sophisticated dynamics packages
* RBDL gives you access to its internals and provides only a thin abstraction layer over the actual computation
* RBDL aims to be self-contained and dependant on as few libraries as possible

And this is what RBDL is ***not*** about:

* RBDL is ***not*** a fully fledged simulator with collision detection or fancy graphics
* RBDL does not keep you from screwing up things.

Multibody dynamics is a complicated subject and in this codebase the
preference is mathematical and algorithmic clarity over elegant software
architecture.

## Licensing

RBDL is published under the very permissive zlib license that gives you a
lot of freedom in the use of full library or parts of it. The core part
of the library is solely using this license but addons may use different
licenses. 

There is no formal contributor license agreement for this project. Instead
when you submit patches or create a pull request it is assumed that you
have the rights to transfer the corresponding code to the RBDL project and
that you are okay that the code will be published as part of RBDL.

## Data Storage

RBDL tries to avoid dynamic allocations and prefers contiguous memory
storage such as in ```std::vector```s over possibly fragmented memory as in
```std::list``` or heap allocated tree structures.

Where possible we use the Structure-of-Arrays (SOA) approach to store data,
e.g. the velocities v of all bodies is stored in an array (```std::vector```)
of ```SpatialVector```s in the ```Model``` structure.

## Naming Conventions

1. Structs and classes use CamelCase, e.g. ```ConstraintSet```
2. Struct and class members use the lowerCamelCase convention, e.g.
  ```Model::dofCount```.
  Exceptions are:
    1. The member variable is a mathematical symbol in an
    algorithm reference, E.g. ```S``` is commonly used to denote the joint
    motion subspace, then we use the algorithm notation. For mathematical
    symbols we also allow the underscore ```_``` to denote a subscript.
    2. Specializations of existing variables may be prefixed with an identifier,
    followed by a underscore. E.g. ```Model::S``` is the default storage for
    joint motion subspaces, however for the specialized 3-DOF joints it uses
    the prefix ```multdof3_``` and are therefore stored in
    ```Model::multdof3_S```.
3. Only the first letter of an acronym is using a capital letter, e.g.
  degree of freedom (DOF) would be used as ```jointDofCount```, or
  ```dofCount```.
4. Variables that are not member variables use the ```snake_case``` convention.

### Examples

    struct Model {
      std::vector<SpatialVector> v;          // ok, v is an  
      std::vector<SpatialVector> S;          // ok, S is commonly used in a reference algorithm
      std::vector<double> u;                 // ok
      std::vector<Vector3d> multdof3_u;      // ok, 3-dof specialization of Model::u

      std::vector<unsigned int> mJointIndex; // NOT OK: invalid prefix
      unsigned int DOFCount;                 // NOT OK: only first letter of abbreviation should be in upper case
      double error_tol;                      // NOT OK: use camelCase instead of snake_case
      void CalcPositions();                  // NOT OK: camelCase for member variables and function must start with lower-case name

    };

## Spacing and Line Width

We use spaces to indent code and use two spaces to indent a block. Do not
use tabs. Namespaces should not be indented.

Lines should not exceed 80 characters in width.

Hint: in the root directory of the RBDL repository you find the file
.editorconfig which uses the correct spacing settings automatically for
many editors (see http://editorconfig.org/ for more details).

## Error Handling

RBDL will fail loudly and abort if an error occurs. This allows you to spot
errors early on. RBDL does not use exceptions.

Code must compile without warnings with all compiler warnings enabled.
Please also consider checking code with static code analyzers such as
clang-analyzer (http://clang-analyzer.llvm.org/).

## Const Correctness

This code uses const correctness, i.e. parameters that are not expected to
change must be specified as const. Use const references whenever possible.
For some optional variables we use pointers, but when possible use
references.

## Braces

Use braces whenever possible. E.g. even there is only a single line of code
after an if-statement wrap it with curly braces. The opening brace starts
in the same line as the ```if``` or the ```else``` statement.

## Documentation

Most importantly the code should be readable to someone who is familiar
with multibody dynamics, especially with Featherstone's notation. The
documentation should mainly serve to clarify the API in terms of doxygen
comments. Within the code itself comments may be used to emphasize on ideas
behind it or to clarify sections. But in general it is best to write
readable code in the first place as comments easily become deprecated.

The doxygen comments should be written in the header files and not in the
```.cc``` files.

## Testing

All code contributions must provide unit tests. RBDL uses UnitTest++
(https://github.com/unittest-cpp/unittest-cpp) as a testing framework. Many
small tests that check single features are preferred over large tests that
test multiple things simultaneously.

Bugfixes ideally come with a test case that reproduce the bug.

## Branching and Bookmarks

RBDL uses Mercurial (https://mercurial-scm.org) as version control system.
The branching concept of mercurial is different than in git. Git's
branching concept is captured in mercurial using bookmarks
(https://www.mercurial-scm.org/wiki/Bookmarks).

The ```default``` branch is reserved for releases. Bugfixes and
contributions happen in the ```dev``` branch but should have a bookmark
assigned to them.

Please do not create new branches, i.e. do not run ```hg branch
<branchname>```.

### Working on a new feature

The following steps are advised when working on a new feature for RBDL:

1. Clone the official repository.
2. Switch to the ```dev``` branch.
3. Create a bookmark that describes that feature preferably in a single
   word.
4. Commit all changes to this bookmark in the ```dev``` branch.
5. Publish your work online and notify the RBDL maintainer(s).

Here are the commands to perform steps 1.-4.:

    # Step 1: clone official repository
    hg clone https://bitbucket.org/rbdl/rbdl <newfeature> && cd <newfeature>

    # Step 2: switch to the dev branch
    hg update dev

    # Step 3: create a bookmark
    hg bookmark newfeature

    # ...
    # Make changes
    # ...

    # Step 4: commit changes
    hg commit

For step 5 the easiest would be to push your changes to a fork of the
official repository on bitbucket and create a pull request.    

## Debugging

* Todo: mention logging facility
* Todo: mention SimpleMath as a fast-compiling (but slower runtime) linear
algebra package.


