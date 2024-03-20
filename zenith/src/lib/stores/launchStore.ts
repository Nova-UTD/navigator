import { asyncWritable, type WritableLoadable } from '@square/svelte-store';
import { getLaunchList, updateLaunchList, type LaunchListEntry, type Node } from '$lib/api';
import { derived, get } from 'svelte/store';
import { isNodeEqual } from '$lib/utils';

export type LaunchState = {
	launches: LaunchListEntry[];
	selectedLaunch: number;
};

export const launchStore: WritableLoadable<LaunchState> = asyncWritable(
	[],
	async () => {
		const launches = await getLaunchList('../launches');
		return { launches, selectedLaunch: 0 };
	},
	async (newLaunch) => {
		return newLaunch;
	}
);

export const selectedLaunchStore = derived(launchStore, ($launchStore) =>
	$launchStore ? $launchStore.launches[$launchStore.selectedLaunch] : undefined
);

export function addNodes(newNodes: Node[]) {
	const selectedLaunch = get(selectedLaunchStore);
	if (!selectedLaunch) return;

	updateLaunchList({
		path: selectedLaunch.path,
		add_nodes: newNodes
	});

	launchStore.update((state) => ({
		...state,
		launches: state.launches.map((launch, i) =>
			i == state.selectedLaunch ? { ...launch, nodes: [...launch.nodes, ...newNodes] } : launch
		)
	}));
}

export function removeNodes(nodesToRemove: Node[]) {
	const selectedLaunch = get(selectedLaunchStore);
	if (!selectedLaunch) return;
	console.log(nodesToRemove);

	updateLaunchList({
		path: selectedLaunch.path,
		remove_nodes: nodesToRemove
	});

	launchStore.update((state) => ({
		...state,
		launches: state.launches.map((launch, i) =>
			i == state.selectedLaunch
				? {
						...launch,
						nodes: launch.nodes.filter((n) => !nodesToRemove.some((node) => isNodeEqual(n, node)))
					}
				: launch
		)
	}));
}
