import { asyncWritable, derived, get, type WritableLoadable } from '@square/svelte-store';
import {
	getLaunchList,
	updateLaunchList,
	createLaunchFile,
	type LaunchListEntry,
	type Node,
	duplicateLaunchFile
} from '$lib/api';
import { filenameFromPath, isNodeEqual } from '$lib/utils';
import { envStore } from './envStore';

export type LaunchState = {
	launches: LaunchListEntry[];
	selectedLaunchIndex: number;
};

/**
 * Store for the list of launches.
 */
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

/**
 * Refresh the launch list and reset the seletected launch index to 0.
 */
export async function refreshLaunchList() {
	const { launchDir } = get(envStore);
	const launches = await getLaunchList(launchDir);
	launchStore.set({ launches, selectedLaunchIndex: 0 });
}

/**
 * Create a new launch.
 * @param name - The name of the launch
 * @param filename - The filename of the launch
 */
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

/**
 * Duplicate a launch.
 * @param newName - The name of the new launch
 * @param oldFilename - The filename of the launch to duplicate
 * @param newFilename - The filename of the new launch
 * @example
 * duplicateLaunch({ newName: 'new launch', oldFilename: 'old_launch.py', newFilename: 'new_launch.py' })
 */
export async function duplicateLaunch({
	newName,
	oldFilename,
	newFilename
}: {
	newName: string;
	oldFilename: string;
	newFilename: string;
}) {
	const { launchDir } = get(envStore);
	launchStore.update((state) => {
		const existingLaunch = state.launches.find(
			(launch) => filenameFromPath(launch.path) === oldFilename
		);

		if (!existingLaunch) return state;

		const newLaunch = {
			...existingLaunch,
			metadata: { ...existingLaunch.metadata, name: newName },
			path: `${launchDir}/${newFilename}`
		};
		return {
			...state,
			launches: [...state.launches, newLaunch]
		};
	});
	duplicateLaunchFile({
		newName,
		oldPath: `${launchDir}/${oldFilename}`,
		newPath: `${launchDir}/${newFilename}`
	});
}

/**
 * Rename a launch.
 * @param path - The path of the launch to rename
 * @param newName - The new name of the launch
 * @param newFilename - The new filename of the launch
 * @example
 * renameLaunch({ path: '/home/user/launch.py', newName: 'new launch', newFilename: 'new_launch.py' })
 */
export function renameLaunch({
	path,
	newName,
	newFilename
}: {
	path: string;
	newName: string;
	newFilename: string;
}) {
	launchStore.update((state) => {
		const launch = state.launches.find((launch) => launch.path === path);
		if (!launch) return state;

		const newLaunch = { ...launch, metadata: { name: newName } };
		return {
			...state,
			launches: state.launches.map((l) => (l.path === path ? newLaunch : l))
		};
	});

	const { launchDir } = get(envStore);

	updateLaunchList({
		path,
		metadata: { name: newName },
		new_path: `${launchDir}/${newFilename}`
	});
}

/**
 * Set the selected launch index.
 * @param index - The index of the launch to set as selected
 */
export function setSelectedLaunchIndex(index: number) {
	launchStore.update((state) => ({
		...state,
		selectedLaunchIndex: index
	}));
}

/**
 * Store for the currently selected launch.
 */
export const selectedLaunchStore = derived(launchStore, ($launchStore) =>
	$launchStore ? $launchStore.launches[$launchStore.selectedLaunchIndex] : undefined
);

/**
 * Adds nodes to the selected launch.
 * @param newNodes - The nodes to add
 */
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

/**
 * Remove nodes from the selected launch.
 * @param nodesToRemove - The nodes to remove
 */
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
