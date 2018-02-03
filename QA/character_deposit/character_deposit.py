import codecs

def handle_deposit(input_file, output_file, deposit_character):
    with codecs.open(input_file, mode='r') as inf:
        output_content = inf.read()
    deleted_content = output_content.replace(deposit_character, '')

    new_output = ''
    deposit_index = 0
    for i in range(len(output_content)):
        if output_content[i] != deposit_character:
            new_output += '\0'
        else:
            if deposit_index < len(deleted_content):
                new_output += deleted_content[deposit_index]
                deposit_index += 1
            else:
                new_output += '\0'
    with open(output_file, 'w') as f:
        f.write(new_output)