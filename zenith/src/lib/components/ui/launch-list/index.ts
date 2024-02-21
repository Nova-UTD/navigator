import LaunchItem from './launch-item.svelte';
import LaunchList from './launch-list.svelte';

export type LaunchListProps = {
	launches: LaunchItemProps[];
};

export type LaunchItemProps = {
	name: string;
	selected?: boolean;
};

export { LaunchItem, LaunchList };
