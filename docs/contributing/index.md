---
layout: default
title: Contributing
nav_order: 9
has_children: true
---
# Contributing overview
{: .no_toc }
 
## Table of contents
{: .no_toc .text-delta }
 
1. TOC
{:toc}
 
---
 
As an open source project, Navigator is always evolving and improving. We open our arms to anyone who'd like to contribute. To help guide contributors, we've developed detailed Code Guidelines & Standards, as well as documentation and branching guidelines that allow us to have a consistent developer experience regardless of author. This revolves around three main philosophies:
- Inherent Readability
- Consistency
- Maintainability
 
By following these guidelines, we can fulfill these philosophies, which allow us to have the most efficient developer experience possible, along with having a mature and maintainable codebase.
 
## Source Control & Git
- Use the branching strategy described below. Do not commit directly to the main or dev branch. This applies even to simple changes, like fixing a typo!
-  Don't merge your own pull requests to main, dev or release branches. Get the software architect or team lead to review your change and merge it.
-  Commit as often or as infrequently as you like. As long as you're working in a branch (as described below), you can commit even a completely broken change without affecting others.
 
### Branching & Feature Branches
We will maintain both a main branch and a dev branch. main is more stable than dev. When you begin work on a feature or a bugfix, fork the dev branch, implement your change, and create a pull request back to dev once the change is relatively stable and well tested.
 
Start branch names with '`feature_`', but other than that, name them however you want. Make the name descriptive and helpful. Don't name a branch `feature_zcc` - that name is not helpful to someone trying to understand what the branch is for. However, `feature_zed_camera_color` clearly describes what the branch is for.
 
Make sure your commits are meaningful and grouped up. Don't commit your entire change history into a single commit, group them up so you have a timeline of your progress.
 
When we are preparing for a release, we will create a branch off of dev to test and and add bugfixes. This release will start with release_ and end with a version number - for example, `release_1_0`. This allows us to continue merging features into dev while a release is being worked on. Once testing is complete, the release branch will be merged into both main and dev, so that the fixes applied during testing are applied everywhere. The commit in main will be tagged as our new release.
 
 
## Documentation
A cornerstone of good code is maintainability, which relies on understandable documentation. Navigator uses Doxygen to automatically generate high-level documentation from our user-defined documentation. Further documentation on how this works can be found below:

https://doxygen.nl/manual/docblocks.html
 
You can also use the [VSCode Doxygen Generator](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen) to automatically generate proper documentation templates.

Below are the different types of documentation that are required for new contributions for Navigator.
 
### Headers
Each file should have documentation on what the file accomplishes, as well as basic usage information. Below is our sample header comment template.
```
/*
* Package:   package_name
* Filename:  FileName.cpp
* Author:    John Doe
* Email:     john.doe@example.com (Use either school or personal)
* Copyright: 2021, Nova UTD
* License:   MIT License
Description of what this file does, what inputs it takes and what it outputs or accomplishes
*/
```
 
### Function & Inline Documentation
Each function should have a Docblock, which documents a specific segment of code (in this case, a Function). Navigator is mainly built on C++ and Python, so below will be examples of Docblocks for each.
 
Note that documentation MUST appear before the declaration it describes, and with the same indentation.
 
These concepts revolve around three main components:
- Description: Description of what the function does
- `@param`: Input type, name and description that's taken into the function
- `@return`: Return type and description that's returned from the function
 

A good resource for documentation is below.

https://developer.lsst.io/cpp/api-docs.html

#### ***Examples:***

#### **C++**
```
/**
* Sum numbers in a vector.
*
* @param values[vector<double>] Container whose values are summed.
* @return double sum of `values`, or 0.0 if `values` is empty.
*/
double sum(std::vector<double> & const values) {
   ...
}
```   
#### **Python**
```
def map_range(number: int, in_min: int, in_max: int, out_min: int, out_max: int) -> int:
   """! Maps a number from one range to another.
   @param number[int]  The input number to map.
   @param in_min[int]  The minimum value of an input number.
   @param in_max[int]  The maximum value of an input number.
   @param out_min[int] The minimum value of an output number.
   @param out_max[int] The maximum value of an output number.
   @return int         Mapped number.
   """
   ...
```
In cases with no parameters or return (ex: constructors, abstracted functions), you can use the below Docblock
#### **C++**
```
/**
* Sum numbers in a vector.
*/
void main() {
   ...
}
```
#### **Python**
```
def init():
   """! Initializes the program."""
```

 
## READMEs
Each subsystem and package must have a README explaining what the directory accomplishes. An example of how these READMEs should be distributed are below.
```
navigator/
 - ..
 - README.md
 - src/
   - ..
   - README.md
   - planning/
     - ..
     - README.md
     - motion_planner/
       - ..
       - README.md
```
 
## Code Standards and Guidelines
Code standards and guidelines are a set of rules and best practices that help us create cleaner, readable and more maintainable code with minimal errors. This help us keep a cleaner, more consistent codebase. This helps us guarantee better code quality, lower code complexity and make more meaningful contributions over time. This all comes down to having better maintainability over time as we scale up our project.

Prefer readability over performance in most code. The obvious exception to this is code that is going to process a large amount of data (ie, the inner loop of a downsampler). However, much of our code runs relatively infrequently, so small performance gains are not worth obfuscating our code.
 
## Styling
As our stack mainly contains C++ and Python code, we chose two styling guidelines for such, these being [Google's C++ guidelines](https://google.github.io/styleguide/cppguide.html) for C++, and [PEP 8](https://peps.python.org/pep-0008/) for Python. We heavily recommend you read at least an overview of these two guidelines. The main takeways for such are below:
 
- Classes/class names should be in PascalCase
- The names of variables (including function parameters) and data members are all lowercase, with underscores between words (snake_case) (eg: our_variable)
- Constants should be fully uppercase (eg: OUR_CONSTANT)
- Bracket initialization should be done on the same line as initialization
- Indentation should be performed with a singular tab, equivalent to four spaces
- Data members of classes (but not structs) additionally have trailing underscores. (eg: a_local_variable, a_struct_data_member,a_class_data_member_)
- Filenames should be all lowercase and can include underscores (_)
- Place a function's variables in the narrowest scope possible, and initialize variables in the declaration
 
## Variable initialization
Variables should be named in snake_case, and should be as verbose as possible. This allows for inherent readability. Examples of this are as below:
- boolean functions and variables should be something like **is_item_exists**
- integer functions and variables should be something like **num_items**

Names should be meaningful, even if it makes them very long. Good names are the #1 factor in writing readable code! Do NOT write ```int inchs = read(sa, b, BUFSZ);``` - instead write ```int characters_read = read(socket, buffer, buffer_size)```

## Abstraction
Abstract as much as possible! This helps with general code cleanliness and readability.
- Split up as much as possible into different functions, we want to reuse as much code as possible
- Split common code into libraries, and have libraries on the highest level of use (eg. code across subsystems should be put into that level of directory, code across packages should be put into that level of directory)
- Don't reinvent the wheel! If you can use existing code rather than writing your own, you usually should. This saves time and often results in using more mature code than we could write ourselves.
- If you end up writing a general-purpose utility of some sort, write it in its own package so that other packages can require it later on (see voltron_test_utils for an example)
- If you need to work with low-level OS functionality (eg, CAN interfaces), either isolate this part of the code in a safe, RAII (Resource Acquisition Is Initilization - look this up if you're unfamiliar with it), C++-style class, or use a library that provides this same functionality. Don't scatter the magic system calls, manual memory management, etc throughout your code.
 
Live by the **DRY** philisophy - Don't Repeat Yourself (*"Every piece of knowledge must have a single, unambiguous, authoritative representation within a system"*). This means reducing repetition of patterns and code duplication, and avoiding code redundancy and duplication.
 
## Types, typing and type hinting
Especially in Python, we want to make sure we use specific types. This helps us catch more error while building instead of during runtime, makes it easier to understand what a piece of code does through inherant code readability, and makes it easier for us to maintain! For example, we don't want to use auto in C++, as this makes it difficult to infer what the variable actually does, instead specifically define the type. An example in Python of typing and type hinting can be found below.
```
def func(foo: int, bar: str) -> List:
   ques: str = str(foo)
   return [ques,bar]
```
 
## Constants and parameters
All constants and parameters should be moved out of the file, or on the top of files. This allows us to define constants at the highest possible level, and allow better readability. If constants require redefining at any point, they'll need to be moved into the root `param.yaml` file. If not, put them at the top of the file, styled in accordance to our styling guidelines.
 
## Misc
- Everything should be nested under the navigator::package_name namespace
- C++ source files should use a .cpp extension. Header files should use a .hpp extension

