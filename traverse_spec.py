import mujoco as mj

# Load the spec from the sample.xml file
spec = mj.MjSpec.from_file("sample.xml")

# Function that recursively prints all body names
def print_bodies(parent, level=0):
    body = parent.first_body()
    while body:
        print(''.join(['-' for i in range(level)]) + body.name)
        print_bodies(body, level + 1)
        body = parent.next_body(body)

print("The spec has the following actuators:")
for actuator in spec.actuators:
    print(actuator.name)

print("\nThe spec has the following bodies:")
print_bodies(spec.worldbody)