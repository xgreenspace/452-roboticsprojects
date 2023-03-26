from rosbag2_py import Player, StorageOptions, PlayOptions


def main(args=None):
    #Play the bag using a launch file, then run the subscriber node
    bag_path = './src/project3/project3/bags/example1'
    storage_options = StorageOptions(uri=bag_path)
    play_options = PlayOptions()
    play_options.loop = False

    player = Player()
    player.play(storage_options, play_options)


if __name__ == '__main__':
    main()