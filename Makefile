register_submodule_updates:
	git pull
	git submodule update --init --recursive
	git submodule foreach git checkout main
	git submodule foreach git pull origin main
	git diff --exit-code websitesource || (git add websitesource && git commit -m "Update websitesource submodule.")
render_websitesource: register_submodule_updates
	make -C ./websitesource html
	cp -r ./websitesource/build/* .
render: render_websitesource
local_test_server: render
	python3 -m http.server
publish: render
	git add .
	git commit -m "Complete render of the entire website."
	git push

