# Unit Tests

The tests in this directory are designed to test the functionality of the components of the
PyCRAM package . The tests are written using the `unittest` framework, however, the tests are run using the `pytest` 
framework. This is because `pytest` is more flexible and provides more features than `unittest` as well as the better 
terminal output.

## Running the tests
To run the tests you need to install the `pytest` package. This can be done by running the following command:

```bash
sudo pip3 install pytest
```

Once `pytest` is installed, you can run all tests by running the following command:

```bash
pytest -v test
```
The -v flag is optional and stands for verbose. It will provide more information about the tests that are run.

To run a specific test file, you can run the following command:

```bash
pytest -v test/test_file.py
```

## Writing tests
When writing tests, you should follow the `unittest` framework. This is because `pytest` is able to run `unittest` tests.

If you need a BulletWorld to run the test in, you can import the BulletWorldTestCase from ``bullet_world_testcase.py`` 
and inherit from it. This will provide a BulletWorld object that is reset before and after each test. Furthermore, there 
are a robot and some objects in the world that can be used for testing. These are defined as instance variables in the
BulletWorldTestCase class.

Available Objects via instance variables:
* robot: The robot object that is in the world, by default this is the PR2
* world: The BulletWorld object 
* milk: A milk object that is in the world
* kitchen: A kitchen object that is in the world
* cereal: A cereal object that is in the world