import random
import string


def generate_text(num_of_group):
    output_str = ''
    for i in range(num_of_group):
        group_size = random.randrange(1, 7)
        for j in range(group_size):
            output_str += random.choice(string.ascii_uppercase)
        for j in range(group_size):
            output_str += 'b'

    return output_str


if __name__ == '__main__':
    t = generate_text(20000)
    print(t)
