Import("env", "projenv")

my_flags = env.ParseFlags(env['BUILD_FLAGS'])
defines = {k: v for (k, v) in my_flags.get("CPPDEFINES")}

# access to global build environment
# print("GLOBAL ENV")
# print(env)

# access to project build environment (is used source files in "src" folder)
# print("PROJECT ENV")
# print(projenv.Dump())

#
# Dump build environment (for debug purpose)
# print(env.Dump())
# print(defines.get("VERSION"));

def after_build(source, target, env):
    # print(source[0]);
    # print(target[0]);
    # print(env["PIOENV"]);
    # do some actions
    # after_build(["buildprog"], [".pio/build/nodemcuv2_nystuen/firmware.bin"])
    version = defines.get("VERSION");
    environment = env["PIOENV"];
    firmware = str(source[0]);
    # print(version);
    # print(environment);
    # print(firmware);

    execute_string = ' '.join(["bash", "./after_build_hook.sh", "--firmware", firmware, "--version", version, "--environment", environment]);
    # print(execute_string);
    env.Execute(execute_string);

env.AddPostAction("buildprog", after_build)
