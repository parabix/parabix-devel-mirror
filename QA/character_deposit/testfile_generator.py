import random
import string


def generate_text(num_of_group, target_character):
    string_set = string.ascii_letters.replace(target_character, '')

    output_str = ''
    for i in range(num_of_group):
        group_size = random.randrange(1, 7)
        for j in range(group_size):
            output_str += random.choice(string_set)
        for j in range(group_size):
            output_str += target_character

    return output_str

if __name__ == '__main__':
    t = generate_text(20000, 'b')
    print(t)
