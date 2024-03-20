from gem5.resources.resource import Resource

resource = Resource("riscv-disk-img")

print(f"the resource is available at {resource.get_local_path()}")