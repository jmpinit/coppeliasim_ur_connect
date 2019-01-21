LUA=lua5.1

docs:
	ldoc --fatalwarnings --project v_rep_ur_connect ur_connect

test:
	$(LUA) tests/run_tests.lua

.PHONY: docs test
