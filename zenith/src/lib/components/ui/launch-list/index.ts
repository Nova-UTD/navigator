import LaunchItem from './launch-item.svelte';
import LaunchList from './launch-list.svelte';

export type LaunchListProps = {
	launches: { name: string; selected: boolean }[];
	onLaunchSelect: (index: number) => void;
};

export type LaunchItemProps = {
	name: string;
	selected?: boolean;
	index: number;
	onClick: (index: number) => void;
};

export { LaunchItem, LaunchList };
