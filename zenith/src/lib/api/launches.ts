export type Node = {
	package: string;
	executable: string;
};

export type LaunchListEntry = {
	path: string;
	metadata: {
		name: string;
	};
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

export type UpdateLaunchEntry = {
	path: string;
	metadata?: {
		name: string;
	};
	add_nodes?: Node[];
	remove_nodes?: Node[];
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
