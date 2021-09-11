def clang_format(name, srcs):
    native.genrule(
        name = name,
        srcs = srcs,
        cmd = "clang-format -style=file -i $(SRCS) && touch $@",
        outs = [name + ".out"],
    )

def cppcheck(name, srcs):
    native.genrule(
        name = name,
        srcs = srcs,
        cmd = "cppcheck . && touch $@",
        outs = [name + ".out"],
    )
