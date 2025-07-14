/* Typescript API bindings for /launches requests. */
// TODO: Change to dynamic URL

export type LaunchMetadata = {
	name: string;
};

export type Node = {
	package: string;
	executable: string;
};

export type LaunchListEntry = {
	path: string;
	metadata: LaunchMetadata;
	nodes: Node[];
};

export type LaunchListResponse = LaunchListEntry[];

export async function getLaunchList(launchDirectory: string): Promise<LaunchListResponse> {
	const response: Promise<LaunchListResponse> = fetch(
		`http://127.0.0.1:8000/launches?dir=${launchDirectory}`
	).then((r) => r.json());

	return response;
}

export async function createLaunchFile(launch: LaunchListEntry): Promise<void> {
	fetch('http://127.0.0.1:8000/launches', {
		method: 'POST',
		headers: {
			'Content-Type': 'application/json'
		},
		body: JSON.stringify(launch)
	});
}

export type DuplicateLaunchParams = {
	newName: string;
	oldPath: string;
	newPath: string;
};
export async function duplicateLaunchFile({
	newName,
	oldPath,
	newPath
}: DuplicateLaunchParams): Promise<void> {
	const urlParams = `old_path=${oldPath}&new_path=${newPath}&new_name=${newName}`;
	fetch(`http://127.0.0.1:8000/launches/copy?${urlParams}`, { method: 'POST' });
}

export type UpdateLaunchEntry = {
	path: string;
	metadata?: LaunchMetadata;
	add_nodes?: Node[];
	remove_nodes?: Node[];
	new_path?: string;
};

export async function updateLaunchList(launch: UpdateLaunchEntry): Promise<void> {
	fetch('http://127.0.0.1:8000/launches', {
		method: 'PATCH',
		headers: {
			'Content-Type': 'application/json'
		},
		body: JSON.stringify(launch)
	}).then(console.log);
}
