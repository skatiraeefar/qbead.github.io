# SpinWearables.github.io

To download for local edits:

`git clone https://github.com/qbead/qbead.github.io.git`

Then download the submodules (e.g., there is one for the main page with the book).

`make register_submodule_updates`

You can edit `websitesource` as if they were standalone git repositories. Consult their own README files. When you are done (and have commited and pushed all the changes for these sub-repositories), you can run

`make publish`

in order to update the live page. This would perform a `git push` on the entire repo, carrying over any changes of the subrepos.

