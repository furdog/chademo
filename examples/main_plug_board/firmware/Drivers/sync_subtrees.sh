cd "$(git rev-parse --show-toplevel)"

git subtree add --prefix=examples/main_plug_board/firmware/Drivers/liblightmodbus/ https://github.com/Jacajack/liblightmodbus.git master --squash
#git subtree pull --prefix=examples/main_plug_board/firmware/Drivers/liblightmodbus/ https://github.com/Jacajack/liblightmodbus.git master --squash
