## Document outlining coding practices and style guidelines

## Naming
### Variables:
* mixedCase (from PEP8)
* Use descriptive names x = bad, numEngines = good

### Classes:
* CapitalizedWords (from PEP8)

### Modules:
* Module names should match class name when a module contains a single class (most common case) (ex. Rocket.py, Vector.py)
* Module name should be plural or a verb if it holds multiple classes / functions (ex. MeanWindModelling.py, RocketComponents.py)

### Methods/Functions:
* Use a leading underscore to indicate that a method (or variable) is only intended for internal use (private)

### Underscores:
* Use underscores when a clear separation is required in a name
* Used most commonly in this repository to separate a common prefix from unique suffixes in names (ex. RigidBody_3DoF / RigidBody_6DoF)

## Formatting
* An auto-formatter is not being used yet. May switch to one in the future  
* Try to wrap long lines (> 120 characters) . For long/verbose function calls/definitions, put each argument on its own line  
* In math, whenever possible, use spacing to hint at the order of operations, Ex: `(x*x + y*y)` is preferred over `(x * x + y * y)`  
* No blank lines immediately after control flow statements

## Other
### Unit Tests:
* Modules should be in ./test/
* Module names should begin with test_
* Class names should begin with Test (and inherit from unittest.TestCase)
* Test method names should begin with test_

### Notes:
* Try and keep code as modular as possible so that changes and improvements can be made easily. Create/specify concise interfaces. Use abstract base classes (abc.ABC) to specify interfaces
* Use functions and classes to simplify code and make its intentions obvious
