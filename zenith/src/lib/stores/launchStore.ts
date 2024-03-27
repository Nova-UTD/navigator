import { asyncWritable, derived, get, type WritableLoadable } from '@square/svelte-store';
import {
	getLaunchList,
	updateLaunchList,
	createLaunchFile,
	type LaunchListEntry,
	type Node
} from '$lib/api';
import { isNodeEqual } from '$lib/utils';
import { envStore } from './envStore';

export type LaunchState = {
	launches: LaunchListEntry[];
	selectedLaunchIndex: number;
};

export const launchStore: WritableLoadable<LaunchState> = asyncWritable(
	[envStore],
	async ([envStore]) => {
		const launches = await getLaunchList(envStore.launchDir);
		return { launches, selectedLaunchIndex: 0 };
	},
	async (newState) => {
		return newState;
	}
);

export function setSelectedLaunchIndex(index: number) {
	launchStore.update((state) => ({
		...state,
		selectedLaunchIndex: index
	}));
}

export const selectedLaunchStore = derived(launchStore, ($launchStore) =>
	$launchStore ? $launchStore.launches[$launchStore.selectedLaunchIndex] : undefined
);

export async function createLaunch(name: string, filename: string) {
	const launchDir = get(envStore).launchDir;
	const path = `${launchDir}/${filename}`;

	launchStore.update((state) => ({
		...state,
		launches: [...state.launches, { metadata: { name }, path, nodes: [] }]
	}));

	createLaunchFile({
		path,
		metadata: { name },
		nodes: []
	});
}

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
			i == state.selectedLaunchIndex ? { ...launch, nodes: [...launch.nodes, ...newNodes] } : launch
		)
	}));
}

export function removeNodes(nodesToRemove: Node[]) {
	const selectedLaunch = get(selectedLaunchStore);
	if (!selectedLaunch) return;

	updateLaunchList({
		path: selectedLaunch.path,
		remove_nodes: nodesToRemove
	});

	launchStore.update((state) => ({
		...state,
		launches: state.launches.map((launch, i) =>
			i == state.selectedLaunchIndex
				? {
						...launch,
						nodes: launch.nodes.filter((n) => !nodesToRemove.some((node) => isNodeEqual(n, node)))
					}
				: launch
		)
	}));
}
