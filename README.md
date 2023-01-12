# O3DE-Extras

o3de/o3de-extras repo is for extra O3DE objects that are considered "canonical" to O3DE but not "core" to the engine. i.e. The engine will build and run without the extras, however the o3de-extras contain "extra" features, projects, gems, etc. in a standard O3DE compound repo format. All extras are tested and maintain the same degree as the core engine and thus can be thought of as part of the engine. When testing is performed on the core engine the extras are included in the testing. Just like the core engine o3de/o3de, the main branch is the stable release branch and is tagged the same way the core engine o3de/o3de main branch is. The development branch is the cutting edge.

## Contribute
For information about contributing to Open 3D Engine, visit [https://o3de.org/docs/contributing/](https://o3de.org/docs/contributing/).

## Download and Register

### Clone the repository 

```shell
git clone https://github.com/o3de/o3de-extras
```

For more details on the steps above, refer to [Setting up O3DE from GitHub](https://o3de.org/docs/welcome-guide/setup/setup-from-github/) in the documentation.

### Setting up o3de-extras

Since the o3de-extras could be cloned anywhere on your local computer, we just need to tell O3DE where to find them by registering them.
From the O3DE repo folder, you can register some or all extra objects using the `o3de register` command.
Since these are all optional objects, we may not need or want all the objects, and we could register a particular object if we wanted, such as a single gem.
You would issue the following command:
```
scripts\o3de.bat register --gem-path <your path to o3de-extras>/Gems/<gem name>
```
Or you may want all the Gems and could register all the Gems. Since all the o3de-extras gems will be in the `<your o3de-extras>/Gems` path you can register all the gem in the extras with one command:
```
scripts\o3de.bat register --all-gems-path <your o3de-extras>/Gems
```
This can be repeated for any object type (if they exist):
```
scripts\o3de.bat register --all-engines-path <your o3de-extras>/Engines
scripts\o3de.bat register --all-projects-path <your o3de-extras>/Projects
scripts\o3de.bat register --all-gems-path <your o3de-extras>/Gems
scripts\o3de.bat register --all-templates-path <your o3de-extras>/Templates
scripts\o3de.bat register --all-restricted-path <your o3de-extras>/Restricted
```
If we registered a gem, which is a piece of a project like a plugin, and we want to use that gem in our project we would only have to tell O3DE to enable that gem for our project by using the `o3de enable-gem` command:
```
scripts\o3de.bat register --enable-gem --gem-name <gem name> --project-name <project name>
```

For a complete tutorial on project configuration, see [Creating Projects Using the Command Line Interface](https://o3de.org/docs/welcome-guide/create/creating-projects-using-cli/) in the documentation.

## License

For terms please see the LICENSE*.TXT files at the root of this distribution.
