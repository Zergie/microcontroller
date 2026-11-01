"""Report linked allocations; stack reservation is not a runtime high-water mark."""
import json
import pathlib
import subprocess
import sys

elf = pathlib.Path(sys.argv[1])
sections = {}
for line in subprocess.check_output(['arm-none-eabi-size', '-A', str(elf)], text=True).splitlines():
    fields = line.split()
    if len(fields) == 3 and fields[0].startswith('.'):
        sections[fields[0]] = (int(fields[1]), int(fields[2]))
flash_end = 0x08000000
headers = subprocess.check_output(['arm-none-eabi-objdump', '-h', str(elf)], text=True).splitlines()
for index, line in enumerate(headers):
    fields = line.split()
    if len(fields) >= 7 and fields[0].isdigit():
        size, load_address = int(fields[2], 16), int(fields[4], 16)
        # NOLOAD sections can inherit a flash LMA but occupy no flash bytes.
        loaded = index + 1 < len(headers) and 'LOAD' in headers[index + 1].split(', ')
        if loaded and size and 0x08000000 <= load_address < 0x08020000:
            flash_end = max(flash_end, load_address + size)
flash = flash_end - 0x08000000
ram = sum(size for size, address in sections.values() if 0x20000000 <= address < 0x20005800)
stack = sections.get('.stack', (0, 0))[0]
ccm_data = sections.get('.ccm', (0, 0))[0]
report = {
    'flash_bytes': flash, 'flash_capacity': 131072,
    'sram_static_bytes': ram, 'sram_capacity': 22528,
    'ccm_stack_reserved_bytes': stack, 'ccm_static_bytes': ccm_data, 'ccm_capacity': 10240,
    'total_ram_reserved_bytes': ram + stack + ccm_data,
    'note': 'Includes static lwIP heap and packet pools. Stack is reserved, not measured at runtime.'
}
if flash > 131072 or ram > 22528 or stack + ccm_data > 10240:
    raise SystemExit('Memory capacity exceeded')
output = elf.parent / 'memory-usage.json'
output.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
