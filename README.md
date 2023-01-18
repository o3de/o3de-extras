# o3de-extras
As the name implies, o3de-extras repo is for "Extra" O3DE objects that are considered "Canonical" to O3DE but not "Core" to the engine.

## O3DE modularity
O3DE is highly modular engine and is essentially a collection of O3DE objects.
O3DE objects are things like projects and gems, discrete objects that can be hierarchical, meaning one object may be the parent or child of another object, some O3DE objects have no children or parents at all.
If an O3DE object has no parent, we say it is a "top level object".
O3DE objects can exist anywhere on your local filesystem, so we do have to tell O3DE where they are by registering all top level objects.
Child objects do not need to be registered as O3DE finds them automatically when the parent is registered.
The result of registration is an entry for each top level object in the `<user>/.o3de/o3de_manifest.json`.
As a matter of organization, child objects are always under the root of the parent object.
For instance the engine itself is a top level object and a parent object.
The engine is the parent of many child objects, such as AutomatedTesting project which lives under its root. 
AutomatedTesting project is the child of the engine object but also is the parent of still other O3DE objects under its root, and has a child gem.
Gems (which are reusable pieces of an o3de project, which other engines might call a plugin) can have other gems as children, and so on. 
Sometimes it is better for an object to not be child, such as when its functionality may not always be needed, or is a community object.

## Core, Canonical and Non-Canonical repos and objects
Canonical means officially supported, tested and part of an official O3DE release.
Non-Canonical means unofficial, untested and not part of an official O3DE release.
Core means it's in the o3de/o3de core engine repo.
Repos, including some o3de.org repos, may or may not be Canonical.
And it follows that any object in a Canonical repo is a Canonical object.
Canonical objects may not rely on any Non-Canonical objects.
Any object in a Non-Canonical repo is considered a Non-Canonical object.
Currently, the core engine (o3de/o3de) and the extras (o3de/o3de-extras) are Canonical repos, and therefore all the object contained within are Canonical.
Canonical repos/objects must meet a few O3DE requirements:

1. They are owned by the O3DE.org/Linux Foundation.
2. They are held to the O3DE coding standard.
3. Their content is managed by a O3DE Special Interest Group (SIG).
That SIG controls that Canonical object and has passes judgement on what is and is not included by that object. 
4. All additions, changes to existing Canonical objects must pass an automated review (AR) in which multiple maintainers code review and approve the change.
The AR system pulls all Canonical repos, registers all Canonical objects, enable all Canonical gems for the AutomatedTesting project, configures, builds and executes all tests.
Everything must pass before it can be accepted into a Canonical repo.
5. All Canonical repos are by definition part of an O3DE release and so must maintain at least 2 branches, a main branch and a development branch.
The main branch is the stable release which is tagged.
The development branch is the cutting edge and is branched for stabilization for a release.
Once stabilized, the stabilization branch is merged into main and tagged for the release.
All changes to stabilization changes are also merged back into development branch.

## Core engine objects (i.e. the objects in o3de/o3de) have an additional requirement:
Anything included in the core must build and not rely on anything outside the core.
So with just the o3de/o3de core repo, the engine and AutomatedTesting project must build.
So if an object is needed or the engine will fail to build, then it is by definition a core object and MUST be in the o3de/o3de core engine repo. 
What we don't want is to have too many core objects.
We want the core to be as small as possible, so that it can run on the widest range of devices possible.
So consideration of the object size and purpose/usefulness should go into the decision on whether the object belongs in the core, extras or another repo.
What objects are excepted into the core should be held to a higher bar than objects in the extras.
Some objects may not be needed to build the engine or AutomatedTesting project but may be considered so useful or "core" to the engine we decide to just include it in the core.
The objects in the core should represent core functionality and should be useful to nearly anyone using O3DE.

## Core or Extras or somewhere else?
How do we know where to put a new object? 
If an object's functionality is optional, this is a good indication it may not be suitable for the core and more likely should be in the extras or another repo.
If the object is large, such as a large project which can be many gigabytes in size, this definitely not in the core, most likely not in the extras, and should probably be in its own repo.
Extras should be the default place for new development.
New core objects should be a red flag and there should be debate and justification on why this new object is core.
We can always promote an extras object in to the core if we want, and we can always demote a core object into the extras.
Once an object has outlived its usefulness it should be demoted into another Non-Canonical deprecation repo.

## New Canonical repos
When we add a new Canonical repo all SIG's must update their processes to include checking those repos for issues just as they do for the core engine repo.
So caution should be taken in how many repos we consider Canonical, as this increases load on the SIGs.

## Testing
All Canonical repos like o3de/o3de-extras and o3de/o3de, use O3DE's automated review (AR) system to gate pull requests (PR's).
So o3de-extras objects are tested with the engine and follow the same coding standards and thus should be thought of as part of the engine.
When AR is performed on a PR to the core engine, o3de-extras development branch is also pulled, registered and gems enabled for the AutomatedTesting project by referencing them in the .automatedtesting.json.
Similarly, when AR is performed on a PR to the extras, the core engine development is pulled, registered and gems enabled for the AutomatedTesting project by referencing them in the .automatedtesting.json.
Everything must build and all tests must pass, with multiple maintainers approving code reviews in order to accept the PR.

## Branches
Just like the core engine, the main branch is the stable release branch and is tagged for release, while the development branch is the cutting edge.
When working on the code make a branch of development, make your changes, create your PR, run and pass AR, merge into development.

## Contribute
For more information about contributing to O3DE, visit [https://o3de.org/docs/contributing/](https://o3de.org/docs/contributing/).

## Download and Register o3de-extras

### Clone the repository 

```shell
git clone https://github.com/o3de/o3de-extras
```

For more details on setting up the engine, refer to [Setting up O3DE from GitHub](https://o3de.org/docs/welcome-guide/setup/setup-from-github/) in the documentation.

### Setting up o3de-extras

Since the o3de-extras repo can be cloned anywhere on your local computer, we just need to tell O3DE where to find the extra objects in this repo by registering them.
From the O3DE repo folder, you can register some or all extra objects using the `o3de register` command.
Since these are all optional objects, we may not need or want all the objects.
If we want to register a particular object such as a single gem we would issue the following command:
```
scripts\o3de.bat register --gem-path <o3de-extras>/Gems/<gem name>
```
Or you may want to register all the Gems.
Since this repo follows the [standard O3DE compound repo format](https://github.com/o3de/o3de/wiki/O3DE-Standard-repo-formats) all the o3de-extras gems will be in the `<o3de-extras>/Gems` path.
We can therefore register all the gems in the extras gems path with one command:
```
scripts\o3de.bat register --all-gems-path <o3de-extras>/Gems
```
This can be repeated for any object type (if they exist):
```
scripts\o3de.bat register --all-engines-path <o3de-extras>/Engines
scripts\o3de.bat register --all-projects-path <o3de-extras>/Projects
scripts\o3de.bat register --all-gems-path <o3de-extras>/Gems
scripts\o3de.bat register --all-templates-path <o3de-extras>/Templates
scripts\o3de.bat register --all-restricted-path <o3de-extras>/Restricted
```
If we registered a gem, which is a piece of a project like a plugin, and we want to use that gem in our project we would only have to tell O3DE to enable that gem for our project by using the `o3de enable-gem` command:
```
scripts\o3de.bat enable-gem --gem-name <gem name> --project-name <project name>
```

For a complete tutorial on project configuration, see [Creating Projects Using the Command Line Interface](https://o3de.org/docs/welcome-guide/create/creating-projects-using-cli/) in the documentation.

## License

For terms please see the LICENSE*.TXT files at the root of this distribution.
